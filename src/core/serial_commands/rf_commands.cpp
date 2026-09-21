#include "rf_commands.h"
#include "cJSON.h"
#include "core/sd_functions.h"
#include "core/type_convertion.h" // decimalToHexString
#include "helpers.h"
#include "modules/rf/protocols/rf_config.h"   // RF_DEBUG
#include "modules/rf/protocols/rf_encoder.h"  // rf_tx_protocol, rf_encoder_selftest
#include "modules/rf/protocols/rf_keeloq.h"   // rf_keeloq_selftest
#include "modules/rf/protocols/rf_presets.h"
#include "modules/rf/protocols/rf_registry.h" // rf_find_protocol
#include "modules/rf/rtl_433/rtl_433.h"
#include "modules/rf/rf_scan.h"
#include "modules/rf/rf_send.h"
#include "modules/rf/rf_utils.h"
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <globals.h>

uint32_t rfRxCallback(cmd *c) {
    Command cmd(c);

    Argument rawArg = cmd.getArgument("raw");
    Argument freqArg = cmd.getArgument("frequency");
    bool raw = rawArg.isSet();
    String strFreq = freqArg.getValue();

    float frequency = strFreq.toFloat();
    frequency /= 1000000; // passed as a long int (e.g. 433920000)

    // serialDevice->print("frequency: ");
    // serialDevice->println(frequency);

    String r = "";
    if (raw) {
        r = rfReceiveSignal(frequency, 10, true, true); // raw mode, headless (Serial only)
    } else {
        r = rfReceiveSignal(frequency, 10, false, true); // decoded mode, headless (Serial only)
    }

    if (r.length() == 0) return false;

    serialDevice->println(r);
    return true;
}

uint32_t rfTxCallback(cmd *c) {
    // flipperzero-like cmd  https://docs.flipper.net/development/cli/#wLVht
    // e.g. subghz tx 0000000000200001 868250000 403 10  //
    // https://forum.flipper.net/t/friedland-libra-48249sl-wireless-doorbell-request/4528/20
    //                {hex_key}     {frequency} {te} {count}
    // subghz tx 445533 433920000 174 10

    Command cmd(c);

    Argument keyArg = cmd.getArgument("key");
    Argument freqArg = cmd.getArgument("frequency");
    Argument teArg = cmd.getArgument("te");
    Argument cntArg = cmd.getArgument("count");
    String strKey = keyArg.getValue();
    String strFrequency = freqArg.getValue();
    String strTe = teArg.getValue();
    String strCount = cntArg.getValue();

    uint64_t key = std::stoull(strKey.c_str(), nullptr, 16);
    unsigned long frequency = std::stoul(strFrequency.c_str());
    unsigned int te = std::stoul(strTe.c_str());
    unsigned int count = std::stoul(strCount.c_str());

    unsigned int bits = 24; // TODO: compute from key

    // check valid frequency and init the rf module
    if (!initRfModule("tx", float(frequency / 1000000.0))) return false;

    rfTransmitCode(key, bits, te, 1, count);
    deinitRfModule();
    return true;
}

uint32_t rfTxByNameCallback(cmd *c) {
    // Transmit by protocol NAME (registry identity), e.g.
    //   subghz txp CAME 433920000 12 0xA5A 0 10
    //   subghz txp Linear 433920000 10 0x2A9
    // Single-line counterpart to a `.sub` replay: resolves `rf_find_protocol`
    // and drives the RMT encoder directly, so it round-trips with the decoder
    // (`Protocol: <name>`). Accepts a full 64-bit hex key.
    Command cmd(c);

    String name = cmd.getArgument("protocol").getValue();
    String strFrequency = cmd.getArgument("frequency").getValue();
    String strBits = cmd.getArgument("bits").getValue();
    String strKey = cmd.getArgument("key").getValue();
    String strTe = cmd.getArgument("te").getValue();
    String strRepeat = cmd.getArgument("repeat").getValue();

    const RfProtocolDef *def = rf_find_protocol(name);
    if (def == nullptr) {
        serialDevice->println("unknown protocol: " + name);
        return false;
    }

    uint64_t key = std::stoull(strKey.c_str(), nullptr, 16);
    unsigned long frequency = std::stoul(strFrequency.c_str());
    unsigned int bits = std::stoul(strBits.c_str());
    int te = strTe.length() ? (int)std::stoul(strTe.c_str()) : 0;
    int repeat = strRepeat.length() ? (int)std::stoul(strRepeat.c_str()) : 10;

    if (!initRfModule("tx", float(frequency / 1000000.0))) return false;
    rf_tx_protocol(key, bits, te, def, repeat);
    deinitRfModule();
    return true;
}

#if RF_DEBUG
uint32_t rfSelftestCallback(cmd *c) {
    // Golden encoder self-test (spec-fidelity of the registry defs). Diagnostic
    // only; compiled under RF_DEBUG.
    return rf_encoder_selftest() ? true : false;
}

uint32_t rfKeeloqTestCallback(cmd *c) {
    // KeeLoq route self-test (per-manufacturer hop framing + cipher round-trip).
    // Diagnostic only; compiled under RF_DEBUG.
    return rf_keeloq_selftest() ? true : false;
}

uint32_t rfKeeloqFileTestCallback(cmd *c) {
    // KeeLoq round-trip driven by the real /mfcodes keystore (every manufacturer
    // with its actual key + learning type). Diagnostic only; under RF_DEBUG.
    return rf_keeloq_filetest() ? true : false;
}
#endif

uint32_t rfKeeloqTxCallback(cmd *c) {
    // Emit a KeeLoq rolling-code frame for a manufacturer in the keystore:
    //   subghz keeloqtx <manufacturer> <freq> <button> <serial_hex> <counter> [repeat]
    // Builds the 64-bit code via keeloq_step (reads /mfcodes) and transmits with
    // the dedicated KeeLoq encoder.
    Command cmd(c);
    String mf = cmd.getArgument("manufacturer").getValue();
    String strFreq = cmd.getArgument("frequency").getValue();
    String strBtn = cmd.getArgument("button").getValue();
    String strSerial = cmd.getArgument("serial").getValue();
    String strCnt = cmd.getArgument("counter").getValue();
    String strRepeat = cmd.getArgument("repeat").getValue();

    RfCodes data{};
    data.mf_name = mf;
    data.btn = (uint8_t)std::stoul(strBtn.c_str());
    data.serial = (uint32_t)std::stoul(strSerial.c_str(), nullptr, 16);
    data.cnt = (uint16_t)std::stoul(strCnt.c_str());
    unsigned long frequency = std::stoul(strFreq.c_str());
    int repeat = strRepeat.length() ? (int)std::stoul(strRepeat.c_str()) : 10;

    data.fix = ((uint32_t)data.btn << 28) | data.serial;
    data.Bit = 64;
    data.keeloq_step(0); // assembles data.key from the manufacturer keystore

    char keyHex[32] = {0};
    decimalToHexString(data.key, keyHex);
    serialDevice->println("keeloqtx mf=" + mf + " key=" + String(keyHex));

    if (!initRfModule("tx", float(frequency / 1000000.0))) return false;
    rf_tx_keeloq(data.key, repeat);
    deinitRfModule();
    return true;
}

// --- /mfcodes management on LittleFS (single-line, robust) ------------------
// Writes the KeeLoq manufacturer keystore to LittleFS so it survives without an
// SD card. Each entry is one line: `mf_name;key_hex;type` (matches the parser).
uint32_t rfMfcodesAddCallback(cmd *c) {
    Command cmd(c);
    String entry = cmd.getArgument("entry").getValue();
    entry.trim();
    if (entry.length() == 0) {
        serialDevice->println("empty entry");
        return false;
    }
    int sc = 0;
    for (unsigned int i = 0; i < entry.length(); i++)
        if (entry[i] == ';') sc++;
    if (sc != 2) {
        serialDevice->println("invalid (need name;keyhex;type): " + entry);
        return false;
    }
    File f = LittleFS.open("/mfcodes", FILE_APPEND, true);
    if (!f) {
        serialDevice->println("LittleFS open failed");
        return false;
    }
    f.print(entry);
    f.print("\n");
    f.close();
    serialDevice->println("added: " + entry);
    return true;
}

uint32_t rfMfcodesClearCallback(cmd *c) {
    if (LittleFS.exists("/mfcodes")) LittleFS.remove("/mfcodes");
    File f = LittleFS.open("/mfcodes", FILE_WRITE, true);
    if (f) f.close();
    serialDevice->println("/mfcodes cleared (LittleFS)");
    return true;
}

uint32_t rfMfcodesListCallback(cmd *c) {
    File f = LittleFS.open("/mfcodes", FILE_READ);
    if (!f) {
        serialDevice->println("no /mfcodes on LittleFS");
        return false;
    }
    int n = 0;
    while (f.available()) {
        String l = f.readStringUntil('\n');
        l.trim();
        if (l.length()) {
            serialDevice->println(l);
            n++;
        }
    }
    f.close();
    serialDevice->println("total: " + String(n));
    return true;
}

uint32_t rfScanCallback(cmd *c) {
    // subghz scan 433 434

    Command cmd(c);

    Argument startArg = cmd.getArgument("start_frequency");
    Argument stopArg = cmd.getArgument("stop_frequency");
    String startFreqStr = startArg.getValue();
    String stopFreqStr = stopArg.getValue();

    float startFreq = startFreqStr.toFloat();
    float stopFreq = stopFreqStr.toFloat();

    if (startFreq == 0 || stopFreq == 0) {
        serialDevice->println("Invalid frequency range: " + String(startFreq) + " - " + String(stopFreq));
        return false;
    }

    // passed as a long int (e.g. 433920000)
    startFreq /= 1000000;
    stopFreq /= 1000000;

    rf_scan(startFreq, stopFreq, 10 * 1000); // 10s timeout
    return true;
}

uint32_t rfTxFileCallback(cmd *c) {
    // example: subghz tx_from_file plug1_on.sub false

    Command cmd(c);

    Argument filepathArg = cmd.getArgument("filepath");
    Argument hideDefaultUIArg = cmd.getArgument("hideDefaultUI");
    String filepath = filepathArg.getValue();
    String hideDefaultUIStr = hideDefaultUIArg.getValue();
    hideDefaultUIStr.trim();
    // CLI: keep the display clean by default; pass "false" to mirror on screen.
    bool hideDefaultUI = !hideDefaultUIStr.equalsIgnoreCase("false");
    filepath.trim();

    if (filepath.indexOf(".sub") == -1) {
        serialDevice->println("Invalid file");
        return false;
    }

    if (!filepath.startsWith("/")) filepath = "/" + filepath;

    FS *fs;
    if (!getFsStorage(fs)) return false;

    if (!(*fs).exists(filepath)) {
        serialDevice->println("File does not exist");
        return false;
    }

    RfCodes data{};

    return readSubFile(fs, filepath, data) && txSubFile(data, hideDefaultUI);
}

uint32_t rfTxBufferCallback(cmd *c) {
#ifndef LITE_VERSION
    if (!(_setupPsramFs())) return false;

    char *txt = _readFileFromSerial();
    String tmpfilepath = "/tmpramfile"; // TODO: Change to use char *txt directly
    File f = PSRamFS.open(tmpfilepath, FILE_WRITE);
    if (!f) return false;

    f.write((const uint8_t *)txt, strlen(txt));
    f.close();
    free(txt);

    RfCodes data{};

    bool r = readSubFile(&PSRamFS, tmpfilepath, data);

    r = txSubFile(data, true); // CLI: don't pollute the display
    PSRamFS.remove(tmpfilepath);

    return r;
#else
    return false;
#endif
}

uint32_t rfSendCallback(cmd *c) {
    // tasmota json command  https://tasmota.github.io/docs/Tasmota-IR/#sending-ir-commands
    // e.g. RfSend {\"Data\":\"0x447503\",\"Bits\":24,\"Protocol\":1,\"Pulse\":174,\"Repeat\":10}  // on
    // e.g. RfSend {\"Data\":\"0x44750C\",\"Bits\":24,\"Protocol\":1,\"Pulse\":174,\"Repeat\":10}  // off

    Command cmd(c);

    Argument args = cmd.getArgument(0);
    String args_str = args.getValue();
    args_str.trim();
    // serialDevice->println(command);

    JsonDocument jsonDoc;
    if (deserializeJson(jsonDoc, args_str)) {
        serialDevice->println("Failed to parse json");
        serialDevice->println(args_str);
        return false;
    }

    JsonObject args_json = jsonDoc.as<JsonObject>(); // root

    unsigned int bits = 32; // defaults to 32 bits
    String dataStr = "";
    int protocol = 1; // defaults to 1
    int pulse = 0;    // 0 leave the library use the default value depending on protocol
    int repeat = 10;

    if (args_json["Data"].isNull()) {
        serialDevice->println("json missing data field");
        return false;
    } else {
        dataStr = args_json["Data"].as<String>();
    }

    uint64_t data_int = strtoul(dataStr.c_str(), nullptr, 16);
    if (data_int == 0) {
        serialDevice->println("rfSendCallback: invalid data value: 0");
        serialDevice->println(dataStr);
        return false;
    }

    if (!args_json["Bits"].isNull()) bits = args_json["Bits"].as<unsigned int>();

    if (!args_json["Pulse"].isNull()) pulse = args_json["Pulse"].as<int>();

    if (!args_json["Protocol"].isNull()) protocol = args_json["Protocol"].as<int>();

    if (!args_json["Repeat"].isNull()) repeat = args_json["Repeat"].as<int>();

    if (!initRfModule("tx")) return false;

    rfTransmitCode(data_int, bits, pulse, protocol, repeat);

    return true;
}

void createRfRxCommand(Command *rfCmd) {
    Command cmd = rfCmd->addCommand("rx", rfRxCallback);
    cmd.addPosArg("frequency", String(bruceConfigPins.rfFreq).c_str());
    cmd.addFlagArg("raw");
}

void createRfTxCommand(Command *rfCmd) {
    Command cmd = rfCmd->addCommand("tx", rfTxCallback);
    cmd.addPosArg("key", "0");
    cmd.addPosArg("frequency", "433920000");
    cmd.addPosArg("te", "0");
    cmd.addPosArg("count", "10");
}

void createRfTxByNameCommand(Command *rfCmd) {
    Command cmd = rfCmd->addCommand("txp", rfTxByNameCallback);
    cmd.addPosArg("protocol");
    cmd.addPosArg("frequency", "433920000");
    cmd.addPosArg("bits", "24");
    cmd.addPosArg("key", "0");
    cmd.addPosArg("te", "0");
    cmd.addPosArg("repeat", "10");
}

#if RF_DEBUG
void createRfSelftestCommand(Command *rfCmd) {
    rfCmd->addCommand("selftest", rfSelftestCallback);
}

void createRfKeeloqTestCommand(Command *rfCmd) {
    rfCmd->addCommand("keeloqtest", rfKeeloqTestCallback);
}

void createRfKeeloqFileTestCommand(Command *rfCmd) {
    rfCmd->addCommand("keeloqfiletest", rfKeeloqFileTestCallback);
}
#endif

void createRfMfcodesCommand(Command *rfCmd) {
    Command cmd = rfCmd->addCompositeCmd("mfcodes");
    Command addCmd = cmd.addCommand("add", rfMfcodesAddCallback);
    addCmd.addPosArg("entry");
    cmd.addCommand("list", rfMfcodesListCallback);
    cmd.addCommand("clear", rfMfcodesClearCallback);
}

void createRfKeeloqTxCommand(Command *rfCmd) {
    Command cmd = rfCmd->addCommand("keeloqtx", rfKeeloqTxCallback);
    cmd.addPosArg("manufacturer");
    cmd.addPosArg("frequency", "433920000");
    cmd.addPosArg("button", "1");
    cmd.addPosArg("serial", "0");
    cmd.addPosArg("counter", "1");
    cmd.addPosArg("repeat", "10");
}

void createRfScanCommand(Command *rfCmd) {
    Command cmd = rfCmd->addCommand("scan", rfScanCallback);
    cmd.addPosArg("start_frequency");
    cmd.addPosArg("stop_frequency");
}

void createRfTxFileCommand(Command *rfCmd) {
    Command cmd = rfCmd->addCommand("tx_from_file", rfTxFileCallback);
    cmd.addPosArg("filepath");
    cmd.addPosArg("hideDefaultUI", "false");
}

void createRfTxBufferCommand(Command *rfCmd) {
    Command cmd = rfCmd->addCommand("tx_from_buffer", rfTxBufferCallback);
}

uint32_t rtl433SniffCallback(cmd *c) {
    Command cmd(c);
    Argument freqArg = cmd.getArgument("frequency");
    Argument modArg = cmd.getArgument("preset");
    Argument cntArg = cmd.getArgument("count");
    Argument timeoutArg = cmd.getArgument("timeout");

    float freq = 433.92f;
    if (freqArg.isSet() && freqArg.getValue().length() > 0) {
        String sf = freqArg.getValue();
        freq = sf.toFloat();
        if (freq > 10000.0f) freq /= 1000000.0f;
    }

    int preset = (freq > 800.0f) ? RTL433_PRESET_OOK_868 :
                 (freq < 330.0f) ? RTL433_PRESET_OOK_315 :
                 (freq > 330.0f && freq < 360.0f) ? RTL433_PRESET_OOK_345 : RTL433_PRESET_OOK_433;

    if (modArg.isSet()) {
        String m = modArg.getValue();
        m.toLowerCase();
        if (m == "fsk" || m == "2fsk" || m == "fsk17" || m == "fsk17k" || m == "wh65") {
            preset = (freq > 800.0f) ? RTL433_PRESET_FSK_868_17K :
                     (freq < 330.0f) ? RTL433_PRESET_FSK_315_19K : RTL433_PRESET_FSK_433_17K;
        } else if (m == "fsk19" || m == "fsk19k" || m == "toyota") {
            preset = (freq < 330.0f) ? RTL433_PRESET_FSK_315_19K : RTL433_PRESET_FSK_433_19K;
        } else if (m == "gfsk" || m == "gfsk17" || m == "bresser") {
            preset = (freq > 800.0f) ? RTL433_PRESET_GFSK_868_17K :
                     (freq < 330.0f) ? RTL433_PRESET_GFSK_315_19K : RTL433_PRESET_GFSK_433_17K;
        } else if (m == "msk" || m == "wmbus") {
            preset = (freq > 800.0f) ? (abs(freq - 868.30f) < 0.15f ? RTL433_PRESET_MSK_868_S : RTL433_PRESET_MSK_868_T) :
                     RTL433_PRESET_MSK_433_100K;
        } else if (m == "ook") {
            preset = (freq > 800.0f) ? RTL433_PRESET_OOK_868 :
                     (freq < 330.0f) ? RTL433_PRESET_OOK_315 :
                     (freq > 330.0f && freq < 360.0f) ? RTL433_PRESET_OOK_345 : RTL433_PRESET_OOK_433;
        } else if (m == "ook868" || m == "868") preset = RTL433_PRESET_OOK_868;
        else if (m == "fsk868") preset = RTL433_PRESET_FSK_868_17K;
        else if (m == "gfsk868") preset = RTL433_PRESET_GFSK_868_17K;
        else if (m == "wmbust" || m == "mskt" || m == "wmbus_t") preset = RTL433_PRESET_MSK_868_T;
        else if (m == "wmbuss" || m == "msks" || m == "wmbus_s") preset = RTL433_PRESET_MSK_868_S;
        else if (m == "honeywell" || m == "345" || m == "ook345") preset = RTL433_PRESET_OOK_345;
        else if (m == "315" || m == "ook315") preset = RTL433_PRESET_OOK_315;
        else if (m == "fsk315") preset = RTL433_PRESET_FSK_315_19K;
        else if (m == "gfsk315") preset = RTL433_PRESET_GFSK_315_19K;
        else if (m.toInt() > 0 || m == "0") preset = constrain(m.toInt(), 0, RTL433_PRESET_COUNT - 1);
    }

    int maxCount = cntArg.isSet() ? cntArg.getValue().toInt() : 10;
    if (maxCount <= 0) maxCount = 1000000;
    int timeoutSec = timeoutArg.isSet() ? timeoutArg.getValue().toInt() : 30;
    uint32_t startMs = millis();
    uint32_t maxDurationMs = (uint32_t)timeoutSec * 1000;

    Rtl433Engine &engine = Rtl433Engine::instance();
    if (!engine.initRadio(freq, preset)) {
        serialDevice->println("{\"error\":\"radio init failed\"}");
        return false;
    }

    RfRxSession rx;
    if (!rx.begin()) {
        engine.deinitRadio();
        serialDevice->println("{\"error\":\"rx session begin failed\"}");
        return false;
    }
    rx.flush();

    serialDevice->println(String("{\"status\":\"sniffing\",\"frequency\":") + String(freq, 4) +
                          ",\"preset\":\"" + String(rtl433_get_preset_name(preset)) + "\"}");

    int captured = 0;
    while (captured < maxCount && (millis() - startMs < maxDurationMs)) {
        if (serialDevice->available()) {
            char ch = serialDevice->read();
            if (ch == 3 || ch == 27 || ch == 'q' || ch == 'Q') break; // Ctrl+C, ESC, q
        }

        int rssi = -70;
        if (bruceConfigPins.rfModule == CC1101_SPI_MODULE) rssi = ELECHOUSE_cc1101.getRssi();

        // 1. Hardware FIFO packet check (CC1101 FSK / GFSK / MSK)
        Rtl433Reading r;
        if (engine.pollFifo(freq, preset, rssi, r)) {
            engine.addRecent(r);
            engine.logJson(r, engine.sdLoggingEnabled);
            captured++;
            serialDevice->println(r.toJson());
        }

        // 2. Software pulse demodulation (OOK and fallback)
        std::vector<int> durations;
        if (rx.poll(durations)) {
            bool decOk = engine.decode(durations, freq, preset, rssi, r);
            if (decOk) {
                engine.addRecent(r);
                engine.logJson(r, engine.sdLoggingEnabled);
                captured++;
                serialDevice->println(r.toJson());
            }
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    rx.end();
    engine.deinitRadio();
    serialDevice->println(String("{\"status\":\"stopped\",\"packets_captured\":") + String(captured) + "}");
    return true;
}

uint32_t rtl433HopCallback(cmd *c) {
    Command cmd(c);
    Argument timeoutArg = cmd.getArgument("timeout");
    Argument groupArg = cmd.getArgument("group");
    Argument totalTimeArg = cmd.getArgument("total");

    int hopTimeoutSec = timeoutArg.isSet() ? timeoutArg.getValue().toInt() : 10;
    if (hopTimeoutSec < 1) hopTimeoutSec = 1;

    int totalDurationSec = totalTimeArg.isSet() ? totalTimeArg.getValue().toInt() : 120;
    if (totalDurationSec < 1) totalDurationSec = 120;

    int group = RTL433_HOP_433_ALL;
    if (groupArg.isSet()) {
        String g = groupArg.getValue();
        g.toLowerCase();
        if (g == "all" || g == "all_presets") group = RTL433_HOP_ALL_PRESETS;
        else if (g == "weather" || g == "wx") group = RTL433_HOP_WEATHER;
        else if (g == "tpms") group = RTL433_HOP_TPMS;
        else if (g == "meters" || g == "wmbus" || g == "smartmeters") group = RTL433_HOP_METERS;
        else if (g == "868") group = RTL433_HOP_868_ALL;
        else if (g == "315") group = RTL433_HOP_315_ALL;
        else if (g.toInt() >= 0 && g.toInt() < RTL433_HOP_GROUP_COUNT) group = g.toInt();
    }

    std::vector<int> hopList = rtl433_get_hop_presets(group);
    if (hopList.empty()) hopList.push_back(RTL433_PRESET_OOK_433);

    Rtl433Engine &engine = Rtl433Engine::instance();
    size_t currentHopIdx = 0;
    int currentPreset = hopList[0];
    float currentFreq = rtl433_get_preset_def(currentPreset)->default_freq;

    if (!engine.initRadio(currentFreq, currentPreset)) {
        serialDevice->println("{\"error\":\"radio init failed\"}");
        return false;
    }

    RfRxSession rx;
    if (!rx.begin()) {
        engine.deinitRadio();
        serialDevice->println("{\"error\":\"rx session begin failed\"}");
        return false;
    }
    rx.flush();

    serialDevice->println(String("{\"status\":\"hopping\",\"group\":\"") + String(rtl433_get_hop_group_name(group)) +
                          "\",\"hop_timeout_s\":" + String(hopTimeoutSec) + "}");

    uint32_t sessionStartMs = millis();
    uint32_t hopStartMs = sessionStartMs;
    uint32_t hopDurationMs = (uint32_t)hopTimeoutSec * 1000;
    uint32_t totalDurationMs = (uint32_t)totalDurationSec * 1000;
    int captured = 0;

    while (millis() - sessionStartMs < totalDurationMs) {
        if (serialDevice->available()) {
            char ch = serialDevice->read();
            if (ch == 3 || ch == 27 || ch == 'q' || ch == 'Q') break;
        }

        uint32_t now = millis();
        if (now - hopStartMs >= hopDurationMs) {
            currentHopIdx = (currentHopIdx + 1) % hopList.size();
            currentPreset = hopList[currentHopIdx];
            currentFreq = rtl433_get_preset_def(currentPreset)->default_freq;
            engine.switchPreset(currentFreq, currentPreset);
            rx.flush();
            hopStartMs = millis();
            serialDevice->println(String("{\"hop_switch\":\"") + String(rtl433_get_preset_name(currentPreset)) +
                                  "\",\"freq\":" + String(currentFreq, 2) + "}");
        }

        int rssi = -70;
        if (bruceConfigPins.rfModule == CC1101_SPI_MODULE) rssi = ELECHOUSE_cc1101.getRssi();

        // 1. Hardware FIFO packet check (CC1101 FSK / GFSK / MSK)
        Rtl433Reading r;
        if (engine.pollFifo(currentFreq, currentPreset, rssi, r)) {
            engine.addRecent(r);
            engine.logJson(r, engine.sdLoggingEnabled);
            captured++;
            hopStartMs = millis();
            serialDevice->println(r.toJson());
        }

        // 2. Software pulse demodulation (OOK and fallback)
        std::vector<int> durations;
        if (rx.poll(durations)) {
            if (engine.decode(durations, currentFreq, currentPreset, rssi, r)) {
                engine.addRecent(r);
                hopStartMs = millis();
                captured++;
                engine.logJson(r, engine.sdLoggingEnabled);
                serialDevice->println(r.toJson());
            }
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    rx.end();
    engine.deinitRadio();
    serialDevice->println(String("{\"status\":\"stopped\",\"packets_captured\":") + String(captured) + "}");
    return true;
}

uint32_t rtl433ListCallback(cmd *c) {
    Rtl433Engine &engine = Rtl433Engine::instance();
    size_t count = engine.getRecentCount();
    serialDevice->println(String("{\"recent_count\":") + String(count) + ",\"readings\":[");
    for (size_t i = 0; i < count; i++) {
        const Rtl433Reading *r = engine.getRecentAt(i);
        if (r) {
            serialDevice->print("  " + r->toJson());
            if (i + 1 < count) serialDevice->print(",");
            serialDevice->println();
        }
    }
    serialDevice->println("]}");
    return true;
}

uint32_t rtl433DumpCallback(cmd *c) {
    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) {
        serialDevice->println("{\"error\":\"storage not available\"}");
        return false;
    }
    String path = "/rtl433/traffic.json";
    if (!fs->exists(path)) path = "/rtl433_traffic.json";
    if (!fs->exists(path)) {
        serialDevice->println("{\"error\":\"log file not found\"}");
        return false;
    }
    File f = fs->open(path, FILE_READ);
    if (!f) {
        serialDevice->println("{\"error\":\"failed to open log file\"}");
        return false;
    }
    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) serialDevice->println(line);
    }
    f.close();
    return true;
}

uint32_t rtl433ClearCallback(cmd *c) {
    Rtl433Engine &engine = Rtl433Engine::instance();
    engine.clearRecent();
    FS *fs = nullptr;
    if (getFsStorage(fs) && fs != nullptr) {
        if (fs->exists("/rtl433/traffic.json")) fs->remove("/rtl433/traffic.json");
        if (fs->exists("/rtl433_traffic.json")) fs->remove("/rtl433_traffic.json");
    }
    serialDevice->println("{\"status\":\"cleared\"}");
    return true;
}

uint32_t rtl433ReplayCallback(cmd *c) {
    Command cmd(c);
    Argument targetArg = cmd.getArgument("target");
    Argument freqArg = cmd.getArgument("frequency");
    Argument repArg = cmd.getArgument("repeats");

    String target = targetArg.isSet() ? targetArg.getValue() : "0";
    float freq = 0.0f;
    if (freqArg.isSet() && freqArg.getValue().length() > 0) {
        freq = freqArg.getValue().toFloat();
        if (freq > 10000.0f) freq /= 1000000.0f;
    }
    int repeats = repArg.isSet() ? repArg.getValue().toInt() : 5;

    Rtl433Engine &engine = Rtl433Engine::instance();

    // Check if target is a sample keyword (nexus, acurite, honeywell, wh65, bresser, wmbus, ook, fsk, gfsk, msk, etc.)
    if (engine.transmitSample(target, freq, repeats)) {
        serialDevice->println("{\"status\":\"transmitted\",\"sample\":\"" + target + "\",\"frequency\":" + String(freq > 0.0f ? freq : 433.92f, 2) + ",\"repeats\":" + String(repeats) + "}");
        return true;
    }

    int index = target.toInt();
    const Rtl433Reading *r = engine.getRecentAt(index);
    if (!r) {
        serialDevice->println("{\"error\":\"invalid sample or index: " + target + "\"}");
        return false;
    }
    if (engine.replayReading(*r, repeats)) {
        serialDevice->println("{\"status\":\"replayed\",\"protocol\":\"" + r->protocol + "\"}");
        return true;
    }
    serialDevice->println("{\"error\":\"replay failed\"}");
    return false;
}

uint32_t rfTxPowerCallback(cmd *c) {
    Command cmd(c);
    Argument powerArg = cmd.getArgument("power");
    int p = 12;
    bool set = false;
    if (powerArg.isSet() && powerArg.getValue().length() > 0) {
        p = powerArg.getValue().toInt();
        set = true;
    } else if (c) {
        String raw = cmd.toString();
        int idx = raw.indexOf("txpower");
        if (idx == -1) idx = raw.indexOf("power");
        if (idx != -1) {
            String sub = raw.substring(idx);
            int minusIdx = sub.indexOf('-');
            if (minusIdx != -1) {
                int val = sub.substring(minusIdx).toInt();
                if (val <= 12 && val >= -30) {
                    p = val;
                    set = true;
                }
            }
        }
    }
    if (set) {
        bruceConfigPins.setRfTxPower(p);
        serialDevice->println("{\"status\":\"ok\",\"rf_tx_power\":" + String(bruceConfigPins.rfTxPower) + "}");
        return true;
    }
    serialDevice->println("{\"rf_tx_power\":" + String(bruceConfigPins.rfTxPower) + "}");
    return true;
}

void createRfTxPowerCommand(Command *rfCmd) {
    Command cmd = rfCmd->addCommand("txpower,power", rfTxPowerCallback);
    cmd.addPosArg("power", "");
}

uint32_t rtl433TestCallback(cmd *c) {
    String report;
    bool ok = rtl433_selftest(report);
    serialDevice->println(report);
    return ok;
}

uint32_t rfPresetCallback(cmd *c) {
    Command cmd(c);
    Argument nameArg = cmd.getArgument("preset");
    String name = nameArg.isSet() ? nameArg.getValue() : "";

    if (name.length() == 0) {
        serialDevice->println("Sub-GHz Scanner Presets (Freq + Modulation):");
        int count = rf_presets_count();
        for (int i = 0; i < count; i++) {
            const RfPreset *p = rf_preset_at(i);
            if (!p) continue;
            const char *modStr = (p->modulation == 0) ? "2-FSK" :
                                 (p->modulation == 1) ? "GFSK" :
                                 (p->modulation == 4) ? "MSK" : "OOK";
            String line = "  [" + String(i) + "] " + String(p->name) + " - " +
                          String(p->defaultFreq, 2) + " MHz (Mod: " + String(modStr) +
                          ", BW: " + String(p->rxBW, 1) + " kHz)";
            serialDevice->println(line);
        }
        return true;
    }

    const RfPreset *p = nullptr;
    if (isDigit(name[0])) {
        int idx = name.toInt();
        if (idx >= 0 && idx < rf_presets_count()) p = rf_preset_at(idx);
    }
    if (!p) p = rf_find_preset(name);

    if (!p) {
        serialDevice->println("Unknown preset: " + name);
        return false;
    }

    rf_apply_preset(p);
    const char *modStr = (p->modulation == 0) ? "2-FSK" :
                         (p->modulation == 1) ? "GFSK" :
                         (p->modulation == 4) ? "MSK" : "OOK";
    serialDevice->println("Applied preset: " + String(p->name) + " (" + String(p->defaultFreq, 2) + " MHz, " + String(modStr) + ")");
    return true;
}

void createRfPresetCommand(Command *rfCmd) {
    Command cmd = rfCmd->addCommand("preset,presets", rfPresetCallback);
    cmd.addPosArg("preset", "");
}

void createRtl433Command(Command *rfCmd) {
    Command cmd = rfCmd->addCompositeCmd("rtl433");

    Command rxCmd = cmd.addCommand("rx,sniff", rtl433SniffCallback);
    rxCmd.addPosArg("frequency", "433.92");
    rxCmd.addPosArg("preset", "ook");
    rxCmd.addPosArg("count", "10");
    rxCmd.addPosArg("timeout", "30");

    Command hopCmd = cmd.addCommand("hop,hop_sniff", rtl433HopCallback);
    hopCmd.addPosArg("timeout", "10");
    hopCmd.addPosArg("group", "433");
    hopCmd.addPosArg("total", "120");

    cmd.addCommand("list,recent", rtl433ListCallback);
    cmd.addCommand("dump", rtl433DumpCallback);
    cmd.addCommand("clear", rtl433ClearCallback);
    cmd.addCommand("test,selftest", rtl433TestCallback);

    Command replayCmd = cmd.addCommand("replay,tx,emit", rtl433ReplayCallback);
    replayCmd.addPosArg("target", "0");
    replayCmd.addPosArg("frequency", "0");
    replayCmd.addPosArg("repeats", "5");
}

void createRfCommands(SimpleCLI *cli) {
    Command cmd = cli->addCompositeCmd("rf,subghz");

    createRfRxCommand(&cmd);
    createRfTxCommand(&cmd);
    createRfTxByNameCommand(&cmd);
    createRfScanCommand(&cmd);
    createRfPresetCommand(&cmd);
    createRfTxFileCommand(&cmd);
    createRfTxBufferCommand(&cmd);
    createRfMfcodesCommand(&cmd);
    createRfKeeloqTxCommand(&cmd);
    createRfTxPowerCommand(&cmd);
    createRtl433Command(&cmd);
#if RF_DEBUG
    createRfSelftestCommand(&cmd);
    createRfKeeloqTestCommand(&cmd);
    createRfKeeloqFileTestCommand(&cmd);
#endif

    cli->addSingleArgCmd("RfSend", rfSendCallback);

    Command topPreset = cli->addCommand("rf_preset,subghz_preset", rfPresetCallback);
    topPreset.addPosArg("preset", "");

    Command topPower = cli->addCommand("txpower,subghz_txpower,rf_txpower", rfTxPowerCallback);
    topPower.addPosArg("power", "");

    // Also register top-level rtl433 command
    Command topRtl = cli->addCompositeCmd("rtl433");
    Command topRx = topRtl.addCommand("rx,sniff", rtl433SniffCallback);
    topRx.addPosArg("frequency", "433.92");
    topRx.addPosArg("preset", "ook");
    topRx.addPosArg("count", "10");
    topRx.addPosArg("timeout", "30");
    Command topHop = topRtl.addCommand("hop,hop_sniff", rtl433HopCallback);
    topHop.addPosArg("timeout", "10");
    topHop.addPosArg("group", "433");
    topHop.addPosArg("total", "120");
    topRtl.addCommand("list,recent", rtl433ListCallback);
    topRtl.addCommand("dump", rtl433DumpCallback);
    topRtl.addCommand("clear", rtl433ClearCallback);
    topRtl.addCommand("test,selftest", rtl433TestCallback);
    Command topReplay = topRtl.addCommand("replay,tx,emit", rtl433ReplayCallback);
    topReplay.addPosArg("target", "0");
    topReplay.addPosArg("frequency", "0");
    topReplay.addPosArg("repeats", "5");
}
