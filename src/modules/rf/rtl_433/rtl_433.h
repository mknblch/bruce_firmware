// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../protocols/rf_decoder.h"
#include "../protocols/rf_encoder.h"
#include "../rf_utils.h"
#include "../structs.h"
#include <Arduino.h>
#include <vector>

// ===========================================================================
// RTL433 Subsystem for Bruce
// Decodes popular Sub-GHz sensor protocols (Weather, TPMS, Security, Remotes)
// on CC1101 (and single-pin receivers) over OOK and 2-FSK modulations.
// ===========================================================================

enum Rtl433Preset {
    RTL433_PRESET_OOK_433 = 0,    // 433.92 MHz OOK (Nexus, Acurite, Oregon, Kerui, Proove, DSC, Schrader)
    RTL433_PRESET_FSK_433_17K,    // 433.92 MHz 2-FSK 17.24kbps (Fine Offset WH65/WH24/WS1000, LaCrosse)
    RTL433_PRESET_FSK_433_19K,    // 433.92 MHz 2-FSK 19.2kbps (Toyota TPMS, FSK TPMS)
    RTL433_PRESET_GFSK_433_17K,   // 433.92 MHz GFSK 17.24kbps (Bresser 5-in-1 / Weather GFSK)
    RTL433_PRESET_MSK_433_100K,   // 433.92 MHz MSK 100kbps (433M MSK / Telemetry)
    RTL433_PRESET_OOK_868,        // 868.35 MHz OOK (EU Weather OOK)
    RTL433_PRESET_FSK_868_17K,    // 868.35 MHz 2-FSK 17.24kbps (EU Fine Offset, LaCrosse)
    RTL433_PRESET_GFSK_868_17K,   // 868.35 MHz GFSK 17.24kbps (EU Bresser 5/6/7-in-1 Weather)
    RTL433_PRESET_MSK_868_T,      // 868.95 MHz MSK 100kbps (Wireless M-Bus Mode T Smart Meters)
    RTL433_PRESET_MSK_868_S,      // 868.30 MHz MSK 32.768kbps (Wireless M-Bus Mode S Smart Meters)
    RTL433_PRESET_OOK_345,        // 345.00 MHz OOK (Honeywell / Ademco 5800)
    RTL433_PRESET_OOK_315,        // 315.00 MHz OOK (US TPMS, Security)
    RTL433_PRESET_FSK_315_19K,    // 315.00 MHz 2-FSK 19.2kbps (US Toyota TPMS)
    RTL433_PRESET_GFSK_315_19K,   // 315.00 MHz GFSK 19.2kbps (US TPMS GFSK)
    RTL433_PRESET_COUNT
};

enum Rtl433ChangingPreset {
    RTL433_CHANGING_433_ALL = 0,       // 433.92 MHz: OOK + 2-FSK + GFSK + MSK
    RTL433_CHANGING_868_ALL,           // 868 MHz: OOK + 2-FSK + GFSK + MSK Mode T/S
    RTL433_CHANGING_ALL_PRESETS,       // All Presets across all bands and modulations
    RTL433_CHANGING_WEATHER,           // Weather: 433 OOK, 433 FSK, 433 GFSK, 868 OOK, 868 FSK, 868 GFSK
    RTL433_CHANGING_TPMS,              // TPMS: 433 OOK, 433 FSK, 315 OOK, 315 FSK, 315 GFSK
    RTL433_CHANGING_METERS,            // Smart Meters (wM-Bus): 868 MSK Mode T, 868 MSK Mode S, 433 MSK
    RTL433_CHANGING_315_ALL,           // 315.00 MHz: OOK + 2-FSK + GFSK
    RTL433_CHANGING_PRESET_COUNT
};

typedef Rtl433ChangingPreset Rtl433HopGroup;
#define RTL433_HOP_433_ALL RTL433_CHANGING_433_ALL
#define RTL433_HOP_868_ALL RTL433_CHANGING_868_ALL
#define RTL433_HOP_ALL_PRESETS RTL433_CHANGING_ALL_PRESETS
#define RTL433_HOP_WEATHER RTL433_CHANGING_WEATHER
#define RTL433_HOP_TPMS RTL433_CHANGING_TPMS
#define RTL433_HOP_METERS RTL433_CHANGING_METERS
#define RTL433_HOP_315_ALL RTL433_CHANGING_315_ALL
#define RTL433_HOP_GROUP_COUNT RTL433_CHANGING_PRESET_COUNT

struct Rtl433PresetDef {
    const char *name;
    float default_freq;
    int modulation;     // 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 4 = MSK
    float deviation;    // kHz
    float rx_bw;        // kHz
    float data_rate;    // kbps
    const char *desc;
};

const Rtl433PresetDef *rtl433_get_preset_def(int preset);
const char *rtl433_get_preset_name(int preset);
std::vector<int> rtl433_get_changing_presets(int changingPreset);
const char *rtl433_get_changing_preset_name(int changingPreset);
inline std::vector<int> rtl433_get_hop_presets(int hopGroup) { return rtl433_get_changing_presets(hopGroup); }
inline const char *rtl433_get_hop_group_name(int hopGroup) { return rtl433_get_changing_preset_name(hopGroup); }

// ---------------------------------------------------------------------------
// Decoded Telemetry Reading
// ---------------------------------------------------------------------------
struct Rtl433Reading {
    String protocol;          // e.g. "FineOffset-WH2", "Acurite-606TX", "Oregon-v2.1", "FineOffset-WH65"
    String model;             // e.g. "WH2 / Rubicson", "606TX", "THGR122N", "WH65B Station"
    String decoder_name;      // e.g. "Nexus", "Acurite", "Oregon", "FineOffset", "Honeywell"
    uint16_t decoder_id = 0;
    uint32_t device_id = 0;
    int8_t channel = -1;      // -1 if not applicable

    bool has_temp = false;
    float temp_c = 0.0f;
    float temp_f = 0.0f;

    bool has_humidity = false;
    float humidity = 0.0f;

    bool has_pressure = false;
    float pressure_kpa = 0.0f;
    float pressure_psi = 0.0f;

    bool has_battery = false;
    bool battery_ok = true;

    bool has_status = false;
    uint16_t status_flags = 0;
    String status_str = "";

    bool has_wind = false;
    float wind_speed_ms = 0.0f;
    float wind_gust_ms = 0.0f;
    int16_t wind_dir_deg = -1;

    bool has_rain = false;
    float rain_mm = 0.0f;

    bool has_uv = false;
    float uv_index = 0.0f;
    float solar_radiation = 0.0f;

    float frequency = 433.92f;
    String modulation = "OOK";
    int preset_idx = 0;
    int rssi = -70;
    String payload_hex = "";
    int bit_len = 0;
    uint32_t timestamp_ms = 0;

    std::vector<int> raw_durations; // Microsecond pulse durations for replay & raw export

    String toJson() const;
    String toSummaryLine() const;
    String toDetailedText() const;
};

// ---------------------------------------------------------------------------
// BitBuffer: Lightweight bit-oriented container for pulse demodulation
// ---------------------------------------------------------------------------
class BitBuffer {
public:
    static const size_t MAX_BYTES = 64;
    uint8_t data[MAX_BYTES];
    uint16_t num_bits;

    BitBuffer() { clear(); }

    void clear() {
        memset(data, 0, sizeof(data));
        num_bits = 0;
    }

    void push_bit(uint8_t bit) {
        if (num_bits >= MAX_BYTES * 8) return;
        uint16_t byte_idx = num_bits / 8;
        uint8_t bit_idx = 7 - (num_bits % 8);
        if (bit) {
            data[byte_idx] |= (1 << bit_idx);
        } else {
            data[byte_idx] &= ~(1 << bit_idx);
        }
        num_bits++;
    }

    uint8_t get_bit(uint16_t idx) const {
        if (idx >= num_bits) return 0;
        uint16_t byte_idx = idx / 8;
        uint8_t bit_idx = 7 - (idx % 8);
        return (data[byte_idx] >> bit_idx) & 1;
    }

    uint8_t get_byte(uint16_t byte_idx) const {
        if (byte_idx >= MAX_BYTES) return 0;
        return data[byte_idx];
    }

    uint8_t get_nibble(uint16_t nibble_idx) const {
        uint16_t byte_idx = nibble_idx / 2;
        if (byte_idx >= MAX_BYTES) return 0;
        if ((nibble_idx % 2) == 0) {
            return (data[byte_idx] >> 4) & 0x0F;
        } else {
            return data[byte_idx] & 0x0F;
        }
    }

    uint32_t extract_bits(uint16_t start_bit, uint8_t count) const {
        if (count > 32 || count == 0) return 0;
        uint32_t val = 0;
        for (uint8_t i = 0; i < count; i++) {
            val = (val << 1) | get_bit(start_bit + i);
        }
        return val;
    }

    uint64_t extract_bits64(uint16_t start_bit, uint8_t count) const {
        if (count > 64 || count == 0) return 0;
        uint64_t val = 0;
        for (uint8_t i = 0; i < count; i++) {
            val = (val << 1) | get_bit(start_bit + i);
        }
        return val;
    }

    int search_sync(uint32_t pattern, uint8_t pattern_len) const {
        if (pattern_len > 32 || pattern_len > num_bits) return -1;
        uint32_t mask = (pattern_len == 32) ? 0xFFFFFFFF : ((1UL << pattern_len) - 1);
        uint32_t window = 0;
        for (uint16_t i = 0; i < num_bits; i++) {
            window = ((window << 1) | get_bit(i)) & mask;
            if (i >= pattern_len - 1 && window == pattern) {
                return i - pattern_len + 1;
            }
        }
        return -1;
    }

    uint8_t crc8(uint8_t poly, uint8_t init = 0, uint16_t start_bit = 0, uint16_t bit_len = 0) const {
        if (bit_len == 0) bit_len = num_bits - start_bit;
        uint8_t crc = init;
        for (uint16_t i = 0; i < bit_len; i++) {
            uint8_t bit = get_bit(start_bit + i);
            if ((crc ^ (bit ? 0x80 : 0)) & 0x80) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
        return crc;
    }

    uint16_t crc16(uint16_t poly, uint16_t init = 0, uint16_t start_bit = 0, uint16_t bit_len = 0) const {
        if (bit_len == 0) bit_len = num_bits - start_bit;
        uint16_t crc = init;
        for (uint16_t i = 0; i < bit_len; i++) {
            uint8_t bit = get_bit(start_bit + i);
            if ((crc ^ (bit ? 0x8000 : 0)) & 0x8000) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
        return crc;
    }

    uint8_t sum_bytes(uint16_t start_byte, uint16_t count) const {
        uint8_t sum = 0;
        for (uint16_t i = 0; i < count; i++) {
            sum += get_byte(start_byte + i);
        }
        return sum;
    }

    uint8_t sum_nibbles(uint16_t start_nibble, uint16_t count) const {
        uint8_t sum = 0;
        for (uint16_t i = 0; i < count; i++) {
            sum += get_nibble(start_nibble + i);
        }
        return sum;
    }

    String to_hex() const {
        String hex = "";
        uint16_t byte_count = (num_bits + 7) / 8;
        for (uint16_t i = 0; i < byte_count; i++) {
            char buf[3];
            snprintf(buf, sizeof(buf), "%02X", data[i]);
            hex += buf;
        }
        return hex;
    }

    void invert() {
        for (size_t i = 0; i < MAX_BYTES; i++) {
            data[i] = ~data[i];
        }
    }
};

// ---------------------------------------------------------------------------
// Pulse Demodulator Helpers
// ---------------------------------------------------------------------------
bool demod_ppm(const std::vector<int> &durations, int mark_us, int zero_gap_us, int one_gap_us, int tol_pct, BitBuffer &out);
bool demod_pwm(const std::vector<int> &durations, int zero_mark_us, int one_mark_us, int space_us, int tol_pct, BitBuffer &out);
bool demod_pwm_space(const std::vector<int> &durations, int mark_us, int zero_space_us, int one_space_us, int tol_pct, BitBuffer &out);
bool demod_manchester(const std::vector<int> &durations, int half_clock_us, int tol_pct, BitBuffer &out, bool invert = false);
bool demod_pcm_fsk(const std::vector<int> &durations, int bit_period_us, int tol_pct, BitBuffer &out, uint32_t sync_word = 0, uint8_t sync_len = 0);

// ---------------------------------------------------------------------------
// Protocol Decoder Declarations
// ---------------------------------------------------------------------------
bool decode_nexus(const std::vector<int> &durations, Rtl433Reading &out);
bool decode_acurite_606tx(const std::vector<int> &durations, Rtl433Reading &out);
bool decode_acurite_tower(const std::vector<int> &durations, Rtl433Reading &out);
bool decode_oregon_scientific(const std::vector<int> &durations, Rtl433Reading &out);
bool decode_fineoffset_fsk(const std::vector<int> &durations, Rtl433Reading &out);
bool decode_lacrosse_tx(const std::vector<int> &durations, Rtl433Reading &out);
bool decode_honeywell_5800(const std::vector<int> &durations, Rtl433Reading &out);
bool decode_schrader_tpms(const std::vector<int> &durations, Rtl433Reading &out);
bool decode_toyota_tpms(const std::vector<int> &durations, Rtl433Reading &out);
bool decode_kerui_ev1527(const std::vector<int> &durations, Rtl433Reading &out);
bool decode_dsc_security(const std::vector<int> &durations, Rtl433Reading &out);
bool decode_proove_nexa(const std::vector<int> &durations, Rtl433Reading &out);
bool decode_bresser_5in1(const std::vector<int> &durations, Rtl433Reading &out);
bool decode_bresser_6in1(const std::vector<int> &durations, Rtl433Reading &out);
bool decode_wmbus(const std::vector<int> &durations, Rtl433Reading &out);

// Unit test / selftest against synthetic pulse test vectors
bool rtl433_selftest(String &report);

// ---------------------------------------------------------------------------
// Subsystem Engine & Storage API
// ---------------------------------------------------------------------------
class Rtl433Engine {
public:
    static Rtl433Engine &instance();

    bool initRadio(float freq, int preset);
    bool switchPreset(float freq, int preset);
    void deinitRadio();

    // Try all registered decoders for the given modulation & duration pulse train
    bool decode(const std::vector<int> &durations, float freq, int preset, int rssi, Rtl433Reading &reading);

    // Logging & Storage
    bool logJson(const Rtl433Reading &reading, bool sd_enabled = true);
    void addRecent(const Rtl433Reading &reading);
    const std::vector<Rtl433Reading> &getRecent() const { return _recentReadings; }
    void clearRecent() {
        _recentReadings.clear();
        _recentReadings.shrink_to_fit();
    }
    size_t getRecentCount() const { return _recentReadings.size(); }
    const Rtl433Reading *getRecentAt(size_t index) const;

    // Signal Replay & .sub Export
    bool replayReading(const Rtl433Reading &reading, int repeatCount = 0);
    bool transmitSample(const String &sampleType, float freq = 0.0f, int repeats = 5);
    bool saveSubFile(const Rtl433Reading &reading, String *outFilename = nullptr);
    size_t saveAllSubFiles(int *savedCount = nullptr);

    // Stats
    uint32_t getPacketsReceived() const { return _packetsReceived; }
    uint32_t getPacketsDecoded() const { return _packetsDecoded; }
    void resetStats() { _packetsReceived = 0; _packetsDecoded = 0; }

    int currentPreset = RTL433_PRESET_OOK_433;
    float currentFrequency = 433.92f;
    bool isChangingPreset = false;
    int changingPreset = RTL433_CHANGING_433_ALL;
    int hopGroup = RTL433_CHANGING_433_ALL;
    uint32_t hopTimeoutMs = 10000;
    bool hopStayOnSignal = true;
    bool sdLoggingEnabled = false;

    String getActivePresetName() const;

    // Replay settings
    bool replayPreamble = false;     // Preamble enabled only when explicitly requested
    int replayFreqSpread = 0;        // 0 = Off (single freq), 1 = +/-15 kHz (3x), 2 = +/-30 kHz (5x), 3 = +/-50 kHz (3x)
    int replayRepeats = 3;           // Number of frame repetitions
    int replayGapMs = 20;            // Gap between frames in ms

private:
    Rtl433Engine() = default;
    std::vector<Rtl433Reading> _recentReadings;
    static const size_t MAX_RECENT = 25;
    uint32_t _packetsReceived = 0;
    uint32_t _packetsDecoded = 0;
};
