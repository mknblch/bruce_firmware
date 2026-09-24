#if !defined(LITE_VERSION)
#include "LoRaConfig.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/utils.h"
#include <FS.h>
#include <LittleFS.h>

LoRaConfigData loraConfig;

const std::vector<LoRaPreset> kLoRaPresets = {
    // Meshtastic LongFast (Default public mesh channel)
    {"Mesh EU868 LongFast", "Meshtastic", 869.525f, 11, 250.0f, 5, 0x2B, 16},
    {"Mesh US915 LongFast", "Meshtastic", 906.875f, 11, 250.0f, 5, 0x2B, 16},
    {"Mesh 433 LongFast",   "Meshtastic", 433.175f, 11, 250.0f, 5, 0x2B, 16},
    {"Mesh AS923 LongFast", "Meshtastic", 923.000f, 11, 250.0f, 5, 0x2B, 16},
    {"Mesh AU915 LongFast", "Meshtastic", 915.000f, 11, 250.0f, 5, 0x2B, 16},

    // Meshtastic MediumFast / MediumSlow
    {"Mesh EU868 MedFast",  "Meshtastic", 869.525f, 9, 250.0f, 5, 0x2B, 16},
    {"Mesh US915 MedFast",  "Meshtastic", 906.875f, 9, 250.0f, 5, 0x2B, 16},
    {"Mesh 433 MedFast",    "Meshtastic", 433.175f, 9, 250.0f, 5, 0x2B, 16},

    // LoRaWAN Public Channels (Sync 0x34)
    {"LoRaWAN EU868 Ch1",   "LoRaWAN",    868.100f, 7, 125.0f, 5, 0x34, 8},
    {"LoRaWAN EU868 Ch2",   "LoRaWAN",    868.300f, 7, 125.0f, 5, 0x34, 8},
    {"LoRaWAN EU868 Ch3",   "LoRaWAN",    868.500f, 7, 125.0f, 5, 0x34, 8},
    {"LoRaWAN US915 Ch1",   "LoRaWAN",    902.300f, 7, 125.0f, 5, 0x34, 8},
    {"LoRaWAN US915 Ch2",   "LoRaWAN",    902.500f, 7, 125.0f, 5, 0x34, 8},
    {"LoRaWAN 433 Ch1",     "LoRaWAN",    433.175f, 7, 125.0f, 5, 0x34, 8},

    // Bruce Chat / Private Channels (Sync 0x12)
    {"Bruce Chat 434.5",    "Bruce",      434.500f, 9, 31.25f, 8, 0x12, 8},
    {"Bruce Chat 868.0",    "Bruce",      868.000f, 9, 31.25f, 8, 0x12, 8},
    {"Bruce Chat 915.0",    "Bruce",      915.000f, 9, 31.25f, 8, 0x12, 8},
    {"LoRa Flipper 433.92", "Generic",    433.920f, 7, 125.0f, 5, 0x12, 8},
    {"LoRa Flipper 868.35", "Generic",    868.350f, 7, 125.0f, 5, 0x12, 8},
    {"LoRa Flipper 915.00", "Generic",    915.000f, 7, 125.0f, 5, 0x12, 8},
};

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
    doc["LoRa_Name"] = loraConfig.username;
    doc["LoRa_Radio"] = (loraConfig.radioType == LoRaRadioType::SX1262) ? "SX1262" : "SX1276";
    doc["LoRa_PCAP"] = loraConfig.enablePcap;

    serializeJson(doc, file);
    file.close();
}

void selectLoRaPresetMenu() {
    loadLoRaConfig();
    std::vector<Option> options;
    options.reserve(kLoRaPresets.size() + 1);

    for (size_t i = 0; i < kLoRaPresets.size(); i++) {
        const auto &p = kLoRaPresets[i];
        String label = String(p.name);
        options.push_back({label, [&p]() {
            loraConfig.freqMHz = p.freqMHz;
            loraConfig.sf = p.sf;
            loraConfig.bwKHz = p.bwKHz;
            loraConfig.cr = p.cr;
            loraConfig.syncWord = p.syncWord;
            loraConfig.preambleLen = p.preambleLen;
            saveLoRaConfig();
            displaySuccess("Preset Applied: " + String(p.name));
        }});
    }

    loopOptions(options, MENU_TYPE_SUBMENU, "LoRa Presets");
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
    options.push_back({"Username: " + loraConfig.username, changeLoRaUsername});

    loopOptions(options, MENU_TYPE_SUBMENU, "LoRa Parameters");
}

#endif // !LITE_VERSION
