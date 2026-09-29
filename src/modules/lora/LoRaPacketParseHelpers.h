#ifndef __LORA_PACKET_PARSE_HELPERS_H__
#define __LORA_PACKET_PARSE_HELPERS_H__

#include <cstddef>
#include <cstdint>

namespace LoRaPacketParseHelpers {

struct MeshtasticDataFields {
    bool hasPort = false;
    uint32_t port = 0;
    bool hasPayload = false;
    size_t payloadOffset = 0;
    size_t payloadLength = 0;
};

inline bool readVarint32(const uint8_t *data, size_t len, size_t &offset, uint32_t &value) {
    if (!data || offset > len) return false;

    size_t cursor = offset;
    uint32_t decoded = 0;
    for (unsigned int i = 0; i < 5 && cursor < len; i++) {
        const uint8_t byte = data[cursor++];
        if (i == 4 && (byte & 0xF0) != 0) return false;
        decoded |= static_cast<uint32_t>(byte & 0x7F) << (i * 7);
        if ((byte & 0x80) == 0) {
            if (i > 0 && byte == 0) return false;
            offset = cursor;
            value = decoded;
            return true;
        }
    }
    return false;
}

inline bool parseMeshtasticDataFields(
    const uint8_t *data,
    size_t len,
    MeshtasticDataFields &fields
) {
    fields = MeshtasticDataFields();
    if (!data && len != 0) return false;

    size_t offset = 0;
    while (offset < len) {
        uint32_t tag = 0;
        if (!readVarint32(data, len, offset, tag)) return false;
        const uint32_t fieldNumber = tag >> 3;
        const uint8_t wireType = static_cast<uint8_t>(tag & 0x07);
        if (fieldNumber == 0 || fieldNumber > 0x1FFFFFFF) return false;

        if (fieldNumber == 1 && wireType != 0) return false;
        if (fieldNumber == 2 && wireType != 2) return false;

        if (wireType == 0) {
            uint32_t value = 0;
            if (!readVarint32(data, len, offset, value)) return false;
            if (fieldNumber == 1) {
                fields.hasPort = true;
                fields.port = value;
            }
        } else if (wireType == 1) {
            if (len - offset < 8) return false;
            offset += 8;
        } else if (wireType == 2) {
            uint32_t byteLength = 0;
            if (!readVarint32(data, len, offset, byteLength)) return false;
            if (static_cast<size_t>(byteLength) > len - offset) return false;
            if (fieldNumber == 2) {
                fields.hasPayload = true;
                fields.payloadOffset = offset;
                fields.payloadLength = byteLength;
            }
            offset += byteLength;
        } else if (wireType == 5) {
            if (len - offset < 4) return false;
            offset += 4;
        } else {
            return false;
        }
    }
    return true;
}

} // namespace LoRaPacketParseHelpers

#endif