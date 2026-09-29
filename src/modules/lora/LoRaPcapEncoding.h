#ifndef __LORA_PCAP_ENCODING_H__
#define __LORA_PCAP_ENCODING_H__

#include "LoRaConfigHelpers.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace LoRaPcapEncoding {

static const size_t GLOBAL_HEADER_LENGTH = 24;
static const size_t RECORD_HEADER_LENGTH = 16;
static const size_t LORATAP_STANDARD_HEADER_LENGTH = 36;
static const size_t BANDWIDTH_EXTENSION_LENGTH = 8;
static const size_t LORATAP_HEADER_LENGTH = LORATAP_STANDARD_HEADER_LENGTH + BANDWIDTH_EXTENSION_LENGTH;
static const uint32_t SNAP_LENGTH = 65535;
static const uint32_t LINKTYPE_LORATAP = 270;
static const uint64_t MIN_VALID_EPOCH_US = 1577836800000000ULL;
static const uint64_t MAX_PCAP_EPOCH_US = static_cast<uint64_t>(UINT32_MAX) * 1000000ULL + 999999ULL;

inline void writeLe16(uint8_t *buffer, uint16_t value) {
    buffer[0] = static_cast<uint8_t>(value & 0xFF);
    buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

inline void writeLe32(uint8_t *buffer, uint32_t value) {
    buffer[0] = static_cast<uint8_t>(value & 0xFF);
    buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buffer[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    buffer[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

inline void writeBe16(uint8_t *buffer, uint16_t value) {
    buffer[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buffer[1] = static_cast<uint8_t>(value & 0xFF);
}

inline void writeBe32(uint8_t *buffer, uint32_t value) {
    buffer[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
    buffer[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
    buffer[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buffer[3] = static_cast<uint8_t>(value & 0xFF);
}

inline void writeBe64(uint8_t *buffer, uint64_t value) {
    writeBe32(buffer, static_cast<uint32_t>(value >> 32));
    writeBe32(buffer + 4, static_cast<uint32_t>(value & 0xFFFFFFFFULL));
}

inline uint32_t readLe32(const uint8_t *buffer) {
    return static_cast<uint32_t>(buffer[0]) | (static_cast<uint32_t>(buffer[1]) << 8) |
           (static_cast<uint32_t>(buffer[2]) << 16) | (static_cast<uint32_t>(buffer[3]) << 24);
}

inline uint32_t readBe32(const uint8_t *buffer) {
    return (static_cast<uint32_t>(buffer[0]) << 24) | (static_cast<uint32_t>(buffer[1]) << 16) |
           (static_cast<uint32_t>(buffer[2]) << 8) | static_cast<uint32_t>(buffer[3]);
}

inline bool encodeGlobalHeader(uint8_t *buffer, size_t capacity) {
    if (!buffer || capacity < GLOBAL_HEADER_LENGTH) return false;
    writeLe32(buffer, 0xA1B2C3D4);
    writeLe16(buffer + 4, 2);
    writeLe16(buffer + 6, 4);
    writeLe32(buffer + 8, 0);
    writeLe32(buffer + 12, 0);
    writeLe32(buffer + 16, SNAP_LENGTH);
    writeLe32(buffer + 20, LINKTYPE_LORATAP);
    return true;
}

inline bool encodeRecordHeader(
    uint8_t *buffer,
    size_t capacity,
    uint32_t timestampSeconds,
    uint32_t timestampMicroseconds,
    uint32_t capturedLength,
    uint32_t originalLength
) {
    if (!buffer || capacity < RECORD_HEADER_LENGTH || timestampMicroseconds >= 1000000) return false;
    writeLe32(buffer, timestampSeconds);
    writeLe32(buffer + 4, timestampMicroseconds);
    writeLe32(buffer + 8, capturedLength);
    writeLe32(buffer + 12, originalLength);
    return true;
}

inline int8_t clampSignedByte(float value) {
    if (!std::isfinite(value)) return 0;
    if (value <= -128.0f) return -128;
    if (value >= 127.0f) return 127;
    const long rounded = std::lround(value);
    return static_cast<int8_t>(rounded);
}

inline bool encodeLoRaTapHeader(
    uint8_t *buffer,
    size_t capacity,
    float frequencyMHz,
    float bandwidthKHz,
    uint8_t spreadingFactor,
    uint8_t codingRate,
    uint8_t syncWord,
    float rssiDbm,
    float snrDb,
    bool crcOk
) {
    if (!buffer || capacity < LORATAP_HEADER_LENGTH ||
        !LoRaConfigHelpers::isValidFrequencyMHz(frequencyMHz) ||
        !LoRaConfigHelpers::isValidBandwidth(bandwidthKHz) ||
        !LoRaConfigHelpers::isValidSpreadingFactor(spreadingFactor) ||
        !LoRaConfigHelpers::isValidCodingRate(codingRate)) {
        return false;
    }

    const double frequencyHz = std::round(static_cast<double>(frequencyMHz) * 1000000.0);
    const double bandwidthHz = std::round(static_cast<double>(bandwidthKHz) * 1000.0);
    if (frequencyHz > std::numeric_limits<uint32_t>::max() ||
        bandwidthHz > std::numeric_limits<uint32_t>::max()) {
        return false;
    }

    buffer[0] = 1;
    buffer[1] = 0;
    writeBe16(buffer + 2, static_cast<uint16_t>(LORATAP_HEADER_LENGTH));

    // Standard LoRaTap v1 prefix; channel bandwidth is encoded in 125 kHz steps.
    writeBe32(buffer + 4, static_cast<uint32_t>(frequencyHz));
    const uint8_t bandwidthSteps = static_cast<uint8_t>(std::round(bandwidthKHz / 125.0f));
    buffer[8] = (bandwidthSteps >= 1 && bandwidthSteps <= 4 &&
                 std::fabs(bandwidthKHz - bandwidthSteps * 125.0f) <= 0.01f) ? bandwidthSteps : 0;
    buffer[9] = spreadingFactor;

    // LoRaTap RSSI uses an offset/quarter-dB representation; 255 marks unavailable fields.
    if (!std::isfinite(rssiDbm) || !std::isfinite(snrDb)) {
        buffer[10] = 255;
    } else {
        const double packetRssi = snrDb >= 0.0f ? rssiDbm + 139.0 : (rssiDbm + 139.0) * 4.0;
        buffer[10] = static_cast<uint8_t>(std::max(0.0, std::min(254.0, std::round(packetRssi))));
    }
    buffer[11] = 255;
    buffer[12] = 255;
    buffer[13] = static_cast<uint8_t>(clampSignedByte(snrDb * 4.0f));
    buffer[14] = syncWord;

    // Remaining standard v1 fields are unavailable on this receiver and are zeroed.
    writeBe64(buffer + 15, 0);
    writeBe32(buffer + 23, 0);
    buffer[28] = crcOk ? 0x08 : 0x10;
    buffer[29] = codingRate;
    writeBe16(buffer + 30, 0);
    buffer[32] = 0;
    buffer[33] = 0;
    writeBe16(buffer + 34, 0);

    // Private TLV 0x8001 carries exact bandwidth in Hz when the 125 kHz field cannot.
    writeBe16(buffer + 36, 0x8001);
    writeBe16(buffer + 38, 4);
    writeBe32(buffer + 40, static_cast<uint32_t>(bandwidthHz));
    return true;
}

// The fallback is explicitly boot-relative when system time is unavailable.
inline bool timestampToPcap(
    uint64_t wallClockNowUs,
    uint32_t uptimeNowMs,
    uint32_t packetUptimeMs,
    uint32_t &seconds,
    uint32_t &microseconds
) {
    const uint32_t ageMs = uptimeNowMs - packetUptimeMs;
    const uint64_t ageUs = static_cast<uint64_t>(ageMs) * 1000ULL;
    if (wallClockNowUs >= MIN_VALID_EPOCH_US && wallClockNowUs <= MAX_PCAP_EPOCH_US && wallClockNowUs >= ageUs) {
        const uint64_t packetEpochUs = wallClockNowUs - ageUs;
        const uint64_t packetSeconds = packetEpochUs / 1000000ULL;
        if (packetSeconds <= UINT32_MAX) {
            seconds = static_cast<uint32_t>(packetSeconds);
            microseconds = static_cast<uint32_t>(packetEpochUs % 1000000ULL);
            return true;
        }
    }

    seconds = packetUptimeMs / 1000;
    microseconds = (packetUptimeMs % 1000) * 1000;
    return false;
}

template <typename Writer>
bool writeExactly(Writer &writer, const uint8_t *buffer, size_t length) {
    return buffer && writer.write(buffer, length) == length;
}

} // namespace LoRaPcapEncoding

#endif