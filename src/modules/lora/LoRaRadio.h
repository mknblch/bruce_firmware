#ifndef __LORA_RADIO_H__
#define __LORA_RADIO_H__

#if !defined(LITE_VERSION)
#include "LoRaConfig.h"
#include <Arduino.h>
#include <RadioLib.h>

extern volatile bool gLoraPacketReceived;
extern volatile bool gLoraInterruptEnabled;

bool isLoraHardwareConfigured();
bool isLoraModulePresent(bool verbose = false);
bool initLoRaRadio(const LoRaConfigData &cfg, bool rxMode = true);
void stopLoRaRadio();

bool setLoRaFrequency(float freqMHz);
bool setLoRaBandwidth(float bwKHz);
bool setLoRaSpreadingFactor(uint8_t sf);
bool setLoRaSyncWord(uint8_t syncWord);
bool setLoRaCodingRate(uint8_t cr);
bool startLoRaReceive();

bool checkLoRaPacketAvailable();
int readLoRaRawData(
    uint8_t *buffer, size_t maxLen, float &rssi, float &snr, float &freqErr, size_t &packetLen
);
bool transmitLoRaRawData(const uint8_t *buffer, size_t len);
bool transmitLoRaString(const String &str);

int scanLoRaCAD();
uint32_t getLoRaIrqFlags();
float getLoRaInstantRSSI();
float getLoRaTimeOnAir(size_t len);

int getLoraIrqPin();
int getLoraBusyPin();
int getLoraResetPin();
int getLoraCsPin();

#endif // !LITE_VERSION
#endif // __LORA_RADIO_H__
