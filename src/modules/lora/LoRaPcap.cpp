#if !defined(LITE_VERSION)
#include "LoRaPcap.h"
#include "core/sd_functions.h"
#include <LittleFS.h>
#include <SD.h>
#include <sys/time.h>

bool LoRaPcapWriter::begin() {
    active = false;
    failed = false;
    filename = "";
    FS *fs = nullptr;
    if (setupSdCard()) {
        fs = &SD;
    } else {
        fs = &LittleFS;
    }

    if (!fs->exists("/BrucePCAP")) {
        if (!fs->mkdir("/BrucePCAP")) {
            failed = true;
            return false;
        }
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
            failed = true;
            return false;
        }
    }

    file = fs->open(filename.c_str(), FILE_WRITE);
    if (!file) {
        active = false;
        failed = true;
        return false;
    }

    uint8_t globalHeader[LoRaPcapEncoding::GLOBAL_HEADER_LENGTH];
    if (!LoRaPcapEncoding::encodeGlobalHeader(globalHeader, sizeof(globalHeader)) ||
        !LoRaPcapEncoding::writeExactly(file, globalHeader, sizeof(globalHeader))) {
        failed = true;
        file.close();
        return false;
    }
    file.flush();

    active = true;
    return true;
}

bool LoRaPcapWriter::writePacket(const LoRaPacket &pkt) {
    if (!active || !file || failed) return false;
    if (pkt.raw.empty()) return true;
    if (pkt.raw.size() > LoRaPcapEncoding::SNAP_LENGTH - LoRaPcapEncoding::LORATAP_HEADER_LENGTH) {
        failed = true;
        active = false;
        return false;
    }

    uint8_t loraTapHeader[LoRaPcapEncoding::LORATAP_HEADER_LENGTH];
    if (!LoRaPcapEncoding::encodeLoRaTapHeader(
            loraTapHeader, sizeof(loraTapHeader), pkt.freqMHz, pkt.bwKHz, pkt.sf, pkt.cr, pkt.syncWord,
            pkt.rssi, pkt.snr, pkt.crcOk
        )) {
        failed = true;
        active = false;
        return false;
    }

    struct timeval wallClock = {};
    uint64_t wallClockNowUs = 0;
    if (gettimeofday(&wallClock, nullptr) == 0 && wallClock.tv_sec >= 0) {
        wallClockNowUs = static_cast<uint64_t>(wallClock.tv_sec) * 1000000ULL + wallClock.tv_usec;
    }
    uint32_t tsSec = 0;
    uint32_t tsUsec = 0;
    LoRaPcapEncoding::timestampToPcap(wallClockNowUs, millis(), pkt.timestampMs, tsSec, tsUsec);

    const uint32_t totalWireLen = LoRaPcapEncoding::LORATAP_HEADER_LENGTH + pkt.raw.size();
    uint8_t recordHeader[LoRaPcapEncoding::RECORD_HEADER_LENGTH];
    if (!LoRaPcapEncoding::encodeRecordHeader(
            recordHeader, sizeof(recordHeader), tsSec, tsUsec, totalWireLen, totalWireLen
        ) ||
        !LoRaPcapEncoding::writeExactly(file, recordHeader, sizeof(recordHeader)) ||
        !LoRaPcapEncoding::writeExactly(file, loraTapHeader, sizeof(loraTapHeader)) ||
        !LoRaPcapEncoding::writeExactly(file, pkt.raw.data(), pkt.raw.size())) {
        failed = true;
        active = false;
        return false;
    }
    file.flush();
    return true;
}

bool LoRaPcapWriter::end() {
    if (file) {
        file.flush();
        file.close();
    }
    active = false;
    return !failed;
}

bool exportLoRaPacketsToPcap(const std::vector<LoRaPacket> &packets, String &savedPath) {
    savedPath = "";
    if (packets.empty()) return false;

    LoRaPcapWriter writer;
    if (!writer.begin()) return false;

    bool writeSucceeded = true;
    for (const auto &pkt : packets) {
        if (!writer.writePacket(pkt)) {
            writeSucceeded = false;
            break;
        }
    }
    const bool endSucceeded = writer.end();
    if (!writeSucceeded || !endSucceeded) return false;
    savedPath = writer.filename;
    return true;
}

#endif // !LITE_VERSION
