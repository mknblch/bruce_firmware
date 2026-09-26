#if !defined(LITE_VERSION)
#include "LoRaConfig.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/sd_functions.h"
#include "core/utils.h"
#include <FS.h>
#include <LittleFS.h>

LoRaConfigData loraConfig;

const std::vector<LoRaPreset> kLoRaPresets = {
    // Meshtastic EU868
    {"Mesh EU868 LongFast",   "Meshtastic EU868", 869.525f, 11, 250.0f, 5, 0x2B, 16},
    {"Mesh EU868 MedFast",    "Meshtastic EU868", 869.525f,  9, 250.0f, 5, 0x2B, 16},
    {"Mesh EU868 MediumSlow", "Meshtastic EU868", 869.525f, 10, 250.0f, 5, 0x2B, 16},
    {"Mesh EU868 ShortSlow",  "Meshtastic EU868", 869.525f,  8, 250.0f, 5, 0x2B, 16},
    {"Mesh EU868 ShortFast",  "Meshtastic EU868", 869.525f,  7, 250.0f, 5, 0x2B, 16},
    {"Mesh EU868 ShortTurbo", "Meshtastic EU868", 869.525f,  7, 500.0f, 5, 0x2B, 16},

    // Meshtastic US915
    {"Mesh US915 LongFast",   "Meshtastic US915", 906.875f, 11, 250.0f, 5, 0x2B, 16},
    {"Mesh US915 MedFast",    "Meshtastic US915", 906.875f,  9, 250.0f, 5, 0x2B, 16},
    {"Mesh US915 MediumSlow", "Meshtastic US915", 906.875f, 10, 250.0f, 5, 0x2B, 16},
    {"Mesh US915 ShortSlow",  "Meshtastic US915", 906.875f,  8, 250.0f, 5, 0x2B, 16},
    {"Mesh US915 ShortFast",  "Meshtastic US915", 906.875f,  7, 250.0f, 5, 0x2B, 16},
    {"Mesh US915 ShortTurbo", "Meshtastic US915", 906.875f,  7, 500.0f, 5, 0x2B, 16},

    // Meshtastic Other Regions
    {"Mesh 433 LongFast",     "Meshtastic 433 MHz", 433.175f, 11, 250.0f, 5, 0x2B, 16},
    {"Mesh 433 MedFast",      "Meshtastic 433 MHz", 433.175f,  9, 250.0f, 5, 0x2B, 16},
    {"Mesh AS923 LongFast",   "Meshtastic AS923",   923.000f, 11, 250.0f, 5, 0x2B, 16},
    {"Mesh AU915 LongFast",   "Meshtastic AU915",   915.000f, 11, 250.0f, 5, 0x2B, 16},

    // LoRaWAN EU868 (Public Sync 0x34)
    {"LoRaWAN EU868 Ch1",     "LoRaWAN EU868",    868.100f,  7, 125.0f, 5, 0x34,  8},
    {"LoRaWAN EU868 Ch2",     "LoRaWAN EU868",    868.300f,  7, 125.0f, 5, 0x34,  8},
    {"LoRaWAN EU868 Ch3",     "LoRaWAN EU868",    868.500f,  7, 125.0f, 5, 0x34,  8},
    {"LoRaWAN EU867 Ch4",     "LoRaWAN EU868",    867.100f,  7, 125.0f, 5, 0x34,  8},
    {"LoRaWAN EU867 Ch5",     "LoRaWAN EU868",    867.300f,  7, 125.0f, 5, 0x34,  8},
    {"LoRaWAN EU867 Ch6",     "LoRaWAN EU868",    867.500f,  7, 125.0f, 5, 0x34,  8},
    {"LoRaWAN EU867 Ch7",     "LoRaWAN EU868",    867.700f,  7, 125.0f, 5, 0x34,  8},
    {"LoRaWAN EU867 Ch8",     "LoRaWAN EU868",    867.900f,  7, 125.0f, 5, 0x34,  8},

    // LoRaWAN US915 (Public Sync 0x34)
    {"LoRaWAN US915 Ch1",     "LoRaWAN US915",    902.300f,  7, 125.0f, 5, 0x34,  8},
    {"LoRaWAN US915 Ch2",     "LoRaWAN US915",    902.500f,  7, 125.0f, 5, 0x34,  8},
    {"LoRaWAN US915 Ch3",     "LoRaWAN US915",    902.700f,  7, 125.0f, 5, 0x34,  8},
    {"LoRaWAN US915 Ch4",     "LoRaWAN US915",    902.900f,  7, 125.0f, 5, 0x34,  8},
    {"LoRaWAN US915 Ch5",     "LoRaWAN US915",    903.100f,  7, 125.0f, 5, 0x34,  8},
    {"LoRaWAN US915 Ch6",     "LoRaWAN US915",    903.300f,  7, 125.0f, 5, 0x34,  8},
    {"LoRaWAN US915 Ch7",     "LoRaWAN US915",    903.500f,  7, 125.0f, 5, 0x34,  8},
    {"LoRaWAN US915 Ch8",     "LoRaWAN US915",    903.700f,  7, 125.0f, 5, 0x34,  8},

    // LoRaWAN 433 MHz
    {"LoRaWAN 433 Ch1",       "LoRaWAN 433 MHz",  433.175f,  7, 125.0f, 5, 0x34,  8},
};

std::vector<LoRaPreset> loadLoRaPresetsFromStorage() {
    std::vector<LoRaPreset> presets;
    static const char *presetFilePath = "/BruceLoRa/presets.json";
    if (!setupSdCard() || !SD.exists(presetFilePath)) return presets;

    File file = SD.open(presetFilePath, FILE_READ);
    if (!file) return presets;
    if (file.size() == 0 || file.size() > 32768) {
        file.close();
        Serial.println("[LoRa] Presets file has an invalid size");
        return presets;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (error || !doc["presets"].is<JsonArrayConst>()) {
        Serial.println("[LoRa] Failed to read /BruceLoRa/presets.json");
        return presets;
    }

    JsonArrayConst entries = doc["presets"].as<JsonArrayConst>();
    for (JsonVariantConst entry : entries) {
        if (presets.size() >= 128) break;
        if (!entry["name"].is<const char *>() || !entry["category"].is<const char *>() ||
            !entry["freqMHz"].is<float>() || !entry["sf"].is<uint8_t>() ||
            !entry["bwKHz"].is<float>() || !entry["cr"].is<uint8_t>() ||
            !entry["syncWord"].is<uint8_t>() || !entry["preambleLen"].is<uint16_t>()) {
            continue;
        }

        String name = entry["name"].as<String>();
        String category = entry["category"].as<String>();
        const float freqMHz = entry["freqMHz"].as<float>();
        const uint8_t sf = entry["sf"].as<uint8_t>();
        const float bwKHz = entry["bwKHz"].as<float>();
        const uint8_t cr = entry["cr"].as<uint8_t>();
        const uint8_t syncWord = entry["syncWord"].as<uint8_t>();
        const uint16_t preambleLen = entry["preambleLen"].as<uint16_t>();

        if (name.isEmpty() || name.length() > 40 || category.isEmpty() || category.length() > 24 ||
            freqMHz < 100.0f || freqMHz > 1000.0f || sf < 5 || sf > 12 || bwKHz < 7.8f ||
            bwKHz > 500.0f || cr < 5 || cr > 8 || preambleLen == 0) {
            continue;
        }

        presets.push_back({name, category, freqMHz, sf, bwKHz, cr, syncWord, preambleLen});
    }
    return presets;
}

void loadLoRaConfig() {
    if (!LittleFS.exists("/lora_settings.json")) {
        saveLoRaConfig();
        return;
    }

    File file = LittleFS.open("/lora_settings.json", "r");
    if (!file) return;

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) return;

    if (!doc["LoRa_Frequency"].isNull()) {
        double rawFreq = doc["LoRa_Frequency"].as<double>();
        if (rawFreq > 1000000.0) {
            loraConfig.freqMHz = (float)(rawFreq / 1000000.0);
        } else if (rawFreq > 1000.0) {
            loraConfig.freqMHz = (float)(rawFreq / 1000.0);
        } else if (rawFreq > 0.0) {
            loraConfig.freqMHz = (float)rawFreq;
        }
    }

    if (!doc["LoRa_SF"].isNull()) loraConfig.sf = doc["LoRa_SF"].as<uint8_t>();
    if (!doc["LoRa_BW"].isNull()) loraConfig.bwKHz = doc["LoRa_BW"].as<float>();
    if (!doc["LoRa_CR"].isNull()) loraConfig.cr = doc["LoRa_CR"].as<uint8_t>();
    if (!doc["LoRa_SyncWord"].isNull()) loraConfig.syncWord = doc["LoRa_SyncWord"].as<uint8_t>();
    if (!doc["LoRa_Preamble"].isNull()) loraConfig.preambleLen = doc["LoRa_Preamble"].as<uint16_t>();
    if (!doc["LoRa_Power"].isNull()) loraConfig.powerDbm = doc["LoRa_Power"].as<int8_t>();
    if (!doc["LoRa_SX1262_Power"].isNull()) {
        loraConfig.sx1262PowerDbm = doc["LoRa_SX1262_Power"].as<int8_t>();
    } else {
        loraConfig.sx1262PowerDbm = loraConfig.powerDbm;
    }
    if (!doc["LoRa_Scan_Dwell"].isNull()) {
        uint16_t scanDwellMs = doc["LoRa_Scan_Dwell"].as<uint16_t>();
        if (scanDwellMs >= 250 && scanDwellMs <= 5000) loraConfig.scanDwellMs = scanDwellMs;
    }
    if (!doc["LoRa_Name"].isNull()) loraConfig.username = doc["LoRa_Name"].as<String>();

    if (!doc["LoRa_Radio"].isNull()) {
        String rad = doc["LoRa_Radio"].as<String>();
        if (rad.equalsIgnoreCase("SX1262")) {
            loraConfig.radioType = LoRaRadioType::SX1262;
        } else {
            loraConfig.radioType = LoRaRadioType::SX1276;
        }
    }

    if (!doc["LoRa_PCAP"].isNull()) loraConfig.enablePcap = doc["LoRa_PCAP"].as<bool>();
}

void saveLoRaConfig() {
    File file = LittleFS.open("/lora_settings.json", "w");
    if (!file) return;

    JsonDocument doc;
    doc["LoRa_Frequency"] = String(loraConfig.freqMHz, 3);
    doc["LoRa_SF"] = loraConfig.sf;
    doc["LoRa_BW"] = loraConfig.bwKHz;
    doc["LoRa_CR"] = loraConfig.cr;
    doc["LoRa_SyncWord"] = loraConfig.syncWord;
    doc["LoRa_Preamble"] = loraConfig.preambleLen;
    doc["LoRa_Power"] = loraConfig.powerDbm;
    doc["LoRa_SX1262_Power"] = loraConfig.sx1262PowerDbm;
    doc["LoRa_Scan_Dwell"] = loraConfig.scanDwellMs;
    doc["LoRa_Name"] = loraConfig.username;
    doc["LoRa_Radio"] = (loraConfig.radioType == LoRaRadioType::SX1262) ? "SX1262" : "SX1276";
    doc["LoRa_PCAP"] = loraConfig.enablePcap;

    serializeJson(doc, file);
    file.close();
}

bool selectLoRaPresetMenu() {
    loadLoRaConfig();
    bool presetSelected = false;
    std::vector<LoRaPreset> presetProfiles(kLoRaPresets.begin(), kLoRaPresets.end());
    std::vector<LoRaPreset> storedPresets = loadLoRaPresetsFromStorage();
    presetProfiles.insert(presetProfiles.end(), storedPresets.begin(), storedPresets.end());
    std::vector<Option> presetOptions = {
        {"Use Current Settings", [&presetSelected]() { presetSelected = true; }}
    };
    for (const auto &preset : presetProfiles) {
        String family = "G";
        if (preset.category.startsWith("Meshtastic")) family = "M";
        else if (preset.category.startsWith("LoRaWAN")) family = "W";
        else if (preset.category.startsWith("Bruce")) family = "B";
        else if (preset.category.startsWith("Flipper")) family = "F";
        else if (preset.category.startsWith("Waveshare")) family = "S";

        String bandwidth = String(preset.bwKHz, 2);
        while (bandwidth.endsWith("0")) bandwidth.remove(bandwidth.length() - 1);
        if (bandwidth.endsWith(".")) bandwidth.remove(bandwidth.length() - 1);

        char syncWordLabel[3];
        snprintf(syncWordLabel, sizeof(syncWordLabel), "%02X", preset.syncWord);
        String label = family + " " + String(preset.freqMHz, 3) + " SF" + String(preset.sf);
        label += " BW" + bandwidth + " CR4/" + String(preset.cr);
        label += " SW";
        label += syncWordLabel;
        label += " P" + String(preset.preambleLen);

        presetOptions.push_back({label, [preset, &presetSelected]() {
            loraConfig.freqMHz = preset.freqMHz;
            loraConfig.sf = preset.sf;
            loraConfig.bwKHz = preset.bwKHz;
            loraConfig.cr = preset.cr;
            loraConfig.syncWord = preset.syncWord;
            loraConfig.preambleLen = preset.preambleLen;
            saveLoRaConfig();
            presetSelected = true;
            displaySuccess("Preset Applied: " + String(preset.name));
        }});
    }

    int selected = loopOptions(presetOptions, MENU_TYPE_SUBMENU, "Select Sniffer Profile", 0, false, false, 0, true, FP);
    return selected >= 0 && presetSelected;
}

void selectLoRaRadioMenu() {
    loadLoRaConfig();
    std::vector<Option> radioOptions = {
        {"SX1276 / SX1278 (default)", []() {
            loraConfig.radioType = LoRaRadioType::SX1276;
            saveLoRaConfig();
            displaySuccess("SX1276 Selected");
        }},
        {"SX1262 / SX1268 (new)", []() {
            loraConfig.radioType = LoRaRadioType::SX1262;
            saveLoRaConfig();
            displaySuccess("SX1262 Selected");
        }}
    };

    int initial = (loraConfig.radioType == LoRaRadioType::SX1262) ? 1 : 0;
    loopOptions(radioOptions, MENU_TYPE_SUBMENU, "LoRa Chipset", initial);
}

void changeLoRaUsername() {
    loadLoRaConfig();
    tft.fillScreen(bruceConfig.bgColor);
    String username = keyboard(loraConfig.username, 32, "LoRa Username:");
    if (username == "" || username == "\x1B") return;
    loraConfig.username = username;
    saveLoRaConfig();
    displaySuccess("Saved: " + username);
}

void changeLoRaFrequency() {
    loadLoRaConfig();
    tft.fillScreen(bruceConfig.bgColor);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.3f", loraConfig.freqMHz);
    String input = num_keyboard(buf, 12, "Freq in MHz:");
    if (input == "" || input == "\x1B") return;

    float f = input.toFloat();
    if (f < 100.0f || f > 1050.0f) {
        displayError("Invalid Freq (100-1050 MHz)");
        return;
    }

    loraConfig.freqMHz = f;
    saveLoRaConfig();
    displaySuccess("Freq: " + String(f, 3) + " MHz");
}

static void selectLoRaTxPowerMenu() {
    loadLoRaConfig();
    static const int8_t powerPresets[] = {-9, 0, 5, 10, 14, 17, 20, 22};
    static const char *powerLabels[] = {
        "-9 dBm (Minimum)", "0 dBm", "5 dBm", "10 dBm (Low)", "14 dBm (Medium)",
        "17 dBm (Default)", "20 dBm (High)", "22 dBm (SX1262 max)"
    };
    std::vector<Option> powerOptions;
    int selected = 0;
    for (size_t i = 0; i < sizeof(powerPresets) / sizeof(powerPresets[0]); i++) {
        const int8_t powerDbm = powerPresets[i];
        if (powerDbm == loraConfig.sx1262PowerDbm) selected = i;
        powerOptions.push_back({powerLabels[i], [powerDbm]() {
            loraConfig.sx1262PowerDbm = powerDbm;
            saveLoRaConfig();
            displaySuccess("SX1262 TX: " + String((int)powerDbm) + " dBm");
        }});
    }
    loopOptions(powerOptions, MENU_TYPE_SUBMENU, "SX1262 TX Power", selected);
}

static void selectLoRaScanDwellMenu() {
    loadLoRaConfig();
    static const uint16_t dwellPresetsMs[] = {250, 500, 1000, 1500, 2000, 3000, 5000};
    std::vector<Option> dwellOptions;
    int selected = 0;
    for (size_t i = 0; i < sizeof(dwellPresetsMs) / sizeof(dwellPresetsMs[0]); i++) {
        const uint16_t dwellMs = dwellPresetsMs[i];
        if (dwellMs == loraConfig.scanDwellMs) selected = i;
        String label = String(dwellMs) + " ms";
        if (dwellMs == 1500) label += " (Default)";
        dwellOptions.push_back({label, [dwellMs]() {
            loraConfig.scanDwellMs = dwellMs;
            saveLoRaConfig();
            displaySuccess("Scan dwell: " + String(dwellMs) + " ms");
        }});
    }
    loopOptions(dwellOptions, MENU_TYPE_SUBMENU, "Channel Scan Dwell", selected);
}

void customLoRaConfigMenu() {
    loadLoRaConfig();
    std::vector<Option> options;

    options.push_back({"Frequency: " + String(loraConfig.freqMHz, 3) + " MHz", []() {
        changeLoRaFrequency();
    }});

    options.push_back({"Spreading Factor (SF" + String(loraConfig.sf) + ")", []() {
        std::vector<Option> sfOpts;
        for (uint8_t sf = 7; sf <= 12; sf++) {
            sfOpts.push_back({"SF" + String(sf), [sf]() {
                loraConfig.sf = sf;
                saveLoRaConfig();
            }});
        }
        loopOptions(sfOpts, MENU_TYPE_SUBMENU, "Select SF", loraConfig.sf - 7);
    }});

    options.push_back({"Bandwidth (" + String(loraConfig.bwKHz, 1) + " kHz)", []() {
        std::vector<Option> bwOpts = {
            {"31.25 kHz (Bruce Chat)", []() { loraConfig.bwKHz = 31.25f; saveLoRaConfig(); }},
            {"62.50 kHz",              []() { loraConfig.bwKHz = 62.50f; saveLoRaConfig(); }},
            {"125.00 kHz (LoRaWAN)",   []() { loraConfig.bwKHz = 125.00f; saveLoRaConfig(); }},
            {"250.00 kHz (Meshtastic)",[]() { loraConfig.bwKHz = 250.00f; saveLoRaConfig(); }},
            {"500.00 kHz (High Speed)",[]() { loraConfig.bwKHz = 500.00f; saveLoRaConfig(); }}
        };
        loopOptions(bwOpts, MENU_TYPE_SUBMENU, "Select BW");
    }});

    options.push_back({"Coding Rate (4/" + String(loraConfig.cr) + ")", []() {
        std::vector<Option> crOpts = {
            {"4/5 (Standard)", []() { loraConfig.cr = 5; saveLoRaConfig(); }},
            {"4/6",            []() { loraConfig.cr = 6; saveLoRaConfig(); }},
            {"4/7",            []() { loraConfig.cr = 7; saveLoRaConfig(); }},
            {"4/8 (Max FEC)",  []() { loraConfig.cr = 8; saveLoRaConfig(); }}
        };
        loopOptions(crOpts, MENU_TYPE_SUBMENU, "Select CR", loraConfig.cr - 5);
    }});

    options.push_back({"Sync Word (0x" + String(loraConfig.syncWord, HEX) + ")", []() {
        std::vector<Option> swOpts = {
            {"0x12 (Private / Bruce)", []() { loraConfig.syncWord = 0x12; saveLoRaConfig(); }},
            {"0x2B (Meshtastic)",      []() { loraConfig.syncWord = 0x2B; saveLoRaConfig(); }},
            {"0x34 (LoRaWAN Public)",  []() { loraConfig.syncWord = 0x34; saveLoRaConfig(); }},
            {"Custom Hex...",          []() {
                String input = keyboard(String(loraConfig.syncWord, HEX), 4, "Sync Word (Hex):");
                if (input != "" && input != "\x1B") {
                    loraConfig.syncWord = (uint8_t)strtol(input.c_str(), NULL, 16);
                    saveLoRaConfig();
                }
            }}
        };
        loopOptions(swOpts, MENU_TYPE_SUBMENU, "Select Sync Word");
    }});

    options.push_back({"Radio Chipset", selectLoRaRadioMenu});
    options.push_back({"SX1262 TX Power (" + String((int)loraConfig.sx1262PowerDbm) + " dBm)", selectLoRaTxPowerMenu});
    options.push_back({"Scan Dwell (" + String(loraConfig.scanDwellMs) + " ms)", selectLoRaScanDwellMenu});
    options.push_back({"Username: " + loraConfig.username, changeLoRaUsername});

    loopOptions(options, MENU_TYPE_SUBMENU, "LoRa Parameters");
}

#endif // !LITE_VERSION
