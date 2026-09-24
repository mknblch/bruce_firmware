#ifndef __LORA_SNIFFER_H__
#define __LORA_SNIFFER_H__

#if !defined(LITE_VERSION)
#include "LoRaPacket.h"
#include <Arduino.h>
#include <vector>

struct LoRaNodeRecord {
    String address;          // e.g. "!1a2b3c4d", "0x26011234", "BruceNode", "RAW_434.50"
    String displayName;      // Pretty display name
    String protocol;         // "Meshtastic", "LoRaWAN", "Bruce", "RAW"
    float lastRssi = -140.0f;
    float peakRssi = -140.0f;
    float lastSnr = 0.0f;
    uint32_t packetCount = 0;
    uint32_t lastSeenMs = 0;
    std::vector<LoRaPacket> packets; // Ring buffer of last 8 packets per MAC/address
};

void runLoRaSniffer();
void showLoRaPacketInspector(const LoRaPacket &pkt);
void showLoRaNodeInspector(LoRaNodeRecord &node);
void viewLoRaCapturedPackets();
void addPacketToSniffer(const LoRaPacket &pkt);

extern std::vector<LoRaPacket> gLoRaCapturedPackets;
extern std::vector<LoRaNodeRecord> gLoRaNodes;

#endif // !LITE_VERSION
#endif // __LORA_SNIFFER_H__
