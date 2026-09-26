#ifndef __LORA_CONFIG_H__
#define __LORA_CONFIG_H__

#if !defined(LITE_VERSION)
#include <Arduino.h>
#include <ArduinoJson.h>
#include <vector>

enum class LoRaRadioType : uint8_t {
    SX1276 = 0,
    SX1262 = 1
};

struct LoRaPreset {
    const char *name;
    const char *category;
    float freqMHz;
    uint8_t sf;
    float bwKHz;
    uint8_t cr;
    uint8_t syncWord;
    uint16_t preambleLen;
};

struct LoRaConfigData {
    float freqMHz = 868.100f;
    uint8_t sf = 9;
    float bwKHz = 31.25f;
    uint8_t cr = 8;             // 4/8
    uint8_t syncWord = 0x12;    // 0x12: Private/Bruce, 0x2B: Meshtastic, 0x34: LoRaWAN
    uint16_t preambleLen = 8;
    int8_t powerDbm = 17;
    LoRaRadioType radioType = LoRaRadioType::SX1262;
    String username = "BruceNode";
    bool promiscuous = true;
    bool enablePcap = false;
};

extern LoRaConfigData loraConfig;
extern const std::vector<LoRaPreset> kLoRaPresets;

void loadLoRaConfig();
void saveLoRaConfig();
void selectLoRaPresetMenu();
void customLoRaConfigMenu();
void changeLoRaUsername();
void changeLoRaFrequency();
void selectLoRaRadioMenu();

#endif // !LITE_VERSION
#endif // __LORA_CONFIG_H__
