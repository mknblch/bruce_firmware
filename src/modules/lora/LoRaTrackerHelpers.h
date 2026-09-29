#ifndef __LORA_TRACKER_HELPERS_H__
#define __LORA_TRACKER_HELPERS_H__

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace LoRaTrackerHelpers {

inline bool normalizeIdentifier(const char *value, char *normalized, size_t capacity) {
    if (!value || !normalized || capacity == 0) return false;
    size_t length = 0;
    for (const unsigned char *ch = reinterpret_cast<const unsigned char *>(value); *ch; ch++) {
        if (*ch >= 0x80 || !std::isalnum(*ch)) continue;
        if (length + 1 >= capacity) {
            normalized[0] = '\0';
            return false;
        }
        normalized[length++] = static_cast<char>(std::tolower(*ch));
    }
    normalized[length] = '\0';
    return true;
}

inline bool isExactOrSuffix(const char *identifier, const char *target) {
    const size_t identifierLength = std::strlen(identifier);
    const size_t targetLength = std::strlen(target);
    return targetLength > 0 && identifierLength >= targetLength &&
           std::strcmp(identifier + identifierLength - targetLength, target) == 0;
}

inline bool matchesTarget(const char *sender, const char *destination, const char *target) {
    char normalizedTarget[96];
    char normalizedIdentifier[128];
    if (!normalizeIdentifier(target, normalizedTarget, sizeof(normalizedTarget)) || normalizedTarget[0] == '\0') {
        return false;
    }

    const char *identifiers[] = {sender, destination};
    for (const char *identifier : identifiers) {
        if (normalizeIdentifier(identifier, normalizedIdentifier, sizeof(normalizedIdentifier)) &&
            isExactOrSuffix(normalizedIdentifier, normalizedTarget)) {
            return true;
        }
    }
    return false;
}

inline float clampHeadingRssi(float rssi, float noiseFloor) {
    return std::max(rssi, noiseFloor);
}

inline float decayPeakFraction(float peakFraction, uint32_t elapsedMs, float fractionPerSecond) {
    const float elapsedSeconds = static_cast<float>(elapsedMs) / 1000.0f;
    return std::max(0.0f, peakFraction - fractionPerSecond * elapsedSeconds);
}

} // namespace LoRaTrackerHelpers

#endif