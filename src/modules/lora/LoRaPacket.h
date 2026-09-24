#ifndef __LORA_PACKET_H__
#define __LORA_PACKET_H__

#if !defined(LITE_VERSION)
#include <Arduino.h>
#include <vector>

enum class LoRaProtocol : uint8_t {
    RAW = 0,
    MESHTASTIC = 1,
    LORAWAN = 2,
    BRUCE_CHAT = 3
};

struct LoRaPacket {
    uint32_t timestampMs = 0;
    float freqMHz = 0;
    float rssi = 0;
    float snr = 0;
    float freqErrorHz = 0;
    float timeOnAirMs = 0;
    uint8_t sf = 0;
    float bwKHz = 0;
    uint8_t cr = 0;
    uint8_t syncWord = 0;
    bool crcOk = true;

    std::vector<uint8_t> raw;

    LoRaProtocol protocol = LoRaProtocol::RAW;
    String protocolName = "RAW";
    String sender = "";
    String destination = "";
    String summary = "";
    String details = "";
    String payloadAscii = "";
    String appName = "";

    uint32_t packetId = 0;
    uint32_t frameCount = 0;
    uint8_t fPort = 0;
    int hopLimit = -1;
    int hopStart = -1;
    bool wantAck = false;
    bool viaMqtt = false;
    uint8_t channelHash = 0;
    uint8_t mType = 0;
};

void parseLoRaPacket(LoRaPacket &pkt);
String formatHexDump(const uint8_t *data, size_t len, size_t maxBytes = 64);
String formatMacAddress(const uint8_t *mac, size_t len);

#endif // !LITE_VERSION
#endif // __LORA_PACKET_H__
