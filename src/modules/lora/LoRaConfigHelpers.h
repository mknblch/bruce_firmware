#ifndef __LORA_CONFIG_HELPERS_H__
#define __LORA_CONFIG_HELPERS_H__

#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace LoRaConfigHelpers {

static const float MIN_FREQUENCY_MHZ = 100.0f;
static const float MAX_FREQUENCY_MHZ = 1050.0f;

inline bool equalsIgnoreCase(const char *left, const char *right) {
    if (!left || !right) return false;
    while (*left && *right) {
        char a = *left++;
        char b = *right++;
        if (a >= 'a' && a <= 'z') a -= 'a' - 'A';
        if (b >= 'a' && b <= 'z') b -= 'a' - 'A';
        if (a != b) return false;
    }
    return *left == '\0' && *right == '\0';
}

inline bool isValidFrequencyMHz(double value) {
    return std::isfinite(value) && value >= MIN_FREQUENCY_MHZ && value <= MAX_FREQUENCY_MHZ;
}

inline bool normalizeFrequencyMHz(double value, const char *unit, float &frequencyMHz) {
    if (!std::isfinite(value)) return false;

    double normalized = value;
    if (unit && unit[0]) {
        if (equalsIgnoreCase(unit, "MHz")) {
            normalized = value;
        } else if (equalsIgnoreCase(unit, "kHz")) {
            normalized = value / 1000.0;
        } else if (equalsIgnoreCase(unit, "Hz")) {
            normalized = value / 1000000.0;
        } else {
            return false;
        }
    } else if (value >= MIN_FREQUENCY_MHZ && value <= MAX_FREQUENCY_MHZ) {
        normalized = value;
    } else if (value > MAX_FREQUENCY_MHZ && value <= 1050000.0) {
        normalized = value / 1000.0;
    } else if (value > 1050000.0 && value <= 1050000000.0) {
        normalized = value / 1000000.0;
    } else {
        return false;
    }

    if (!isValidFrequencyMHz(normalized)) return false;
    frequencyMHz = static_cast<float>(normalized);
    return true;
}

inline bool normalizeFrequencyMHz(const char *value, const char *unit, float &frequencyMHz) {
    if (!value || !value[0]) return false;
    char *end = nullptr;
    const double numeric = std::strtod(value, &end);
    if (end == value || !std::isfinite(numeric)) return false;
    while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n') end++;
    return *end == '\0' && normalizeFrequencyMHz(numeric, unit, frequencyMHz);
}

inline bool isValidSpreadingFactor(int value) {
    return value >= 5 && value <= 12;
}

inline bool isValidBandwidth(float value) {
    static const float supported[] = {7.8f, 10.4f, 15.6f, 20.8f, 31.25f, 41.7f, 62.5f, 125.0f, 250.0f, 500.0f};
    if (!std::isfinite(value)) return false;
    for (float bandwidth : supported) {
        if (std::fabs(value - bandwidth) <= 0.15f) return true;
    }
    return false;
}

inline bool isValidCodingRate(int value) {
    return value >= 5 && value <= 8;
}

inline bool isValidSyncWord(int value) {
    return value >= 1 && value <= 255;
}

inline bool isValidPreambleLength(int value) {
    return value >= 1 && value <= 65535;
}

inline bool isValidTxPower(int value, bool isSx1262) {
    return isSx1262 ? (value >= -9 && value <= 22) : (value >= 2 && value <= 20);
}

inline bool isValidTcxoVoltage(float value) {
    static const float supported[] = {0.0f, 1.6f, 1.7f, 1.8f, 2.2f, 2.4f, 2.7f, 3.0f, 3.3f};
    if (!std::isfinite(value)) return false;
    for (float voltage : supported) {
        if (std::fabs(value - voltage) <= 0.01f) return true;
    }
    return false;
}

inline bool isValidScanDwell(int value) {
    return value >= 1000 && value <= 5000;
}

inline bool isValidRadioType(int value) {
    return value == 0 || value == 1;
}

inline bool isValidRadioSettings(
    float frequencyMHz,
    int spreadingFactor,
    float bandwidthKHz,
    int codingRate,
    int preambleLength,
    int sx1276PowerDbm,
    int sx1262PowerDbm,
    int scanDwellMs,
    int radioType,
    float sx1262TcxoVoltage = 3.0f
) {
    return isValidFrequencyMHz(frequencyMHz) && isValidSpreadingFactor(spreadingFactor) &&
           isValidBandwidth(bandwidthKHz) && isValidCodingRate(codingRate) &&
           isValidPreambleLength(preambleLength) && isValidTxPower(sx1276PowerDbm, false) &&
           isValidTxPower(sx1262PowerDbm, true) && isValidScanDwell(scanDwellMs) &&
           isValidRadioType(radioType) && isValidTcxoVoltage(sx1262TcxoVoltage);
}

} // namespace LoRaConfigHelpers

#endif