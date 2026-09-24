#ifndef __LORA_PCAP_H__
#define __LORA_PCAP_H__

#if !defined(LITE_VERSION)
#include "LoRaPacket.h"
#include <Arduino.h>
#include <FS.h>

class LoRaPcapWriter {
public:
    File file;
    bool active = false;
    String filename = "";

    bool begin();
    void writePacket(const LoRaPacket &pkt);
    void end();
    bool isActive() const { return active; }
};

bool exportLoRaPacketsToPcap(const std::vector<LoRaPacket> &packets, String &savedPath);

#endif // !LITE_VERSION
#endif // __LORA_PCAP_H__
