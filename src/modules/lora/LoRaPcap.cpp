#if !defined(LITE_VERSION)
#include "LoRaPcap.h"
#include "core/sd_functions.h"
#include <LittleFS.h>
#include <SD.h>

// Global PCAP Header (LINKTYPE_LORATAP = 270)
static const uint32_t PCAP_MAGIC = 0xa1b2c3d4;
static const uint16_t PCAP_VER_MAJOR = 2;
static const uint16_t PCAP_VER_MINOR = 4;
static const uint32_t PCAP_SNAPLEN = 65535;
static const uint32_t PCAP_LINKTYPE_LORATAP = 270;

bool LoRaPcapWriter::begin() {
    FS *fs = nullptr;
    if (setupSdCard()) {
        fs = &SD;
    } else {
        fs = &LittleFS;
    }

    if (!fs->exists("/BrucePCAP")) {
        fs->mkdir("/BrucePCAP");
    }

    int idx = 0;
    while (true) {
        String path = "/BrucePCAP/lora_" + String(idx) + ".pcap";
        if (!fs->exists(path.c_str())) {
            filename = path;
            break;
        }
        idx++;
        if (idx > 999) {
            filename = "/BrucePCAP/lora_999.pcap";
            break;
        }
    }

    file = fs->open(filename.c_str(), FILE_WRITE);
    if (!file) {
        active = false;
        return false;
    }

    // Write Global PCAP Header
    uint32_t thiszone = 0;
    uint32_t sigfigs = 0;
    file.write((const uint8_t *)&PCAP_MAGIC, 4);
    file.write((const uint8_t *)&PCAP_VER_MAJOR, 2);
    file.write((const uint8_t *)&PCAP_VER_MINOR, 2);
    file.write((const uint8_t *)&thiszone, 4);
    file.write((const uint8_t *)&sigfigs, 4);
    file.write((const uint8_t *)&PCAP_SNAPLEN, 4);
    file.write((const uint8_t *)&PCAP_LINKTYPE_LORATAP, 4);
    file.flush();

    active = true;
    return true;
}

void LoRaPcapWriter::writePacket(const LoRaPacket &pkt) {
    if (!active || !file || pkt.raw.empty()) return;

    // LoRaTap Header layout:
    // Byte 0: version = 1
    // Byte 1: padding = 0
    // Byte 2..3: header length (LE uint16) = 4 (base) + 11 (channel TLV) + 6 (RSSI TLV) = 21 bytes
    //
    // TLV 1: Channel Info (Type 0x0001, Len 7)
    // - Type: 0x0001 (uint16 LE)
    // - Len: 7 (uint16 LE)
    // - Freq: uint32 LE (in Hz)
    // - BW: uint8 (in kHz, or enum)
    // - SF: uint8
    // - CR: uint8
    //
    // TLV 2: RSSI & SNR (Type 0x0002, Len 2)
    // - Type: 0x0002 (uint16 LE)
    // - Len: 2 (uint16 LE)
    // - RSSI: int8 (dBm)
    // - SNR: int8 (dB)

    uint8_t loraTapBuf[32];
    uint16_t headerLen = 4 + 11 + 6;

    loraTapBuf[0] = 1; // LoRaTap version 1
    loraTapBuf[1] = 0; // padding
    loraTapBuf[2] = headerLen & 0xFF;
    loraTapBuf[3] = (headerLen >> 8) & 0xFF;

    // TLV 1: Channel
    uint16_t tlv1Type = 0x0001;
    uint16_t tlv1Len = 7;
    uint32_t freqHz = (uint32_t)(pkt.freqMHz * 1000000.0f);
    uint8_t bw = (uint8_t)(pkt.bwKHz);
    uint8_t sf = pkt.sf;
    uint8_t cr = pkt.cr;

    loraTapBuf[4] = tlv1Type & 0xFF;
    loraTapBuf[5] = (tlv1Type >> 8) & 0xFF;
    loraTapBuf[6] = tlv1Len & 0xFF;
    loraTapBuf[7] = (tlv1Len >> 8) & 0xFF;
    loraTapBuf[8] = freqHz & 0xFF;
    loraTapBuf[9] = (freqHz >> 8) & 0xFF;
    loraTapBuf[10] = (freqHz >> 16) & 0xFF;
    loraTapBuf[11] = (freqHz >> 24) & 0xFF;
    loraTapBuf[12] = bw;
    loraTapBuf[13] = sf;
    loraTapBuf[14] = cr;

    // TLV 2: RSSI / SNR
    uint16_t tlv2Type = 0x0002;
    uint16_t tlv2Len = 2;
    int8_t rssiVal = (int8_t)pkt.rssi;
    int8_t snrVal = (int8_t)pkt.snr;

    loraTapBuf[15] = tlv2Type & 0xFF;
    loraTapBuf[16] = (tlv2Type >> 8) & 0xFF;
    loraTapBuf[17] = tlv2Len & 0xFF;
    loraTapBuf[18] = (tlv2Len >> 8) & 0xFF;
    loraTapBuf[19] = (uint8_t)rssiVal;
    loraTapBuf[20] = (uint8_t)snrVal;

    uint32_t totalWireLen = headerLen + pkt.raw.size();

    // PCAP Record Header (16 bytes)
    uint32_t tsSec = pkt.timestampMs / 1000;
    uint32_t tsUsec = (pkt.timestampMs % 1000) * 1000;
    uint32_t inclLen = totalWireLen;
    uint32_t origLen = totalWireLen;

    file.write((const uint8_t *)&tsSec, 4);
    file.write((const uint8_t *)&tsUsec, 4);
    file.write((const uint8_t *)&inclLen, 4);
    file.write((const uint8_t *)&origLen, 4);

    // Write LoRaTap Header + Raw Payload
    file.write(loraTapBuf, headerLen);
    file.write(pkt.raw.data(), pkt.raw.size());
    file.flush();
}

void LoRaPcapWriter::end() {
    if (active && file) {
        file.flush();
        file.close();
    }
    active = false;
}

bool exportLoRaPacketsToPcap(const std::vector<LoRaPacket> &packets, String &savedPath) {
    if (packets.empty()) return false;

    LoRaPcapWriter writer;
    if (!writer.begin()) return false;

    savedPath = writer.filename;
    for (const auto &pkt : packets) {
        writer.writePacket(pkt);
    }
    writer.end();
    return true;
}

#endif // !LITE_VERSION
