// SPDX-License-Identifier: AGPL-3.0-or-later
#include "rtl_433.h"
#include "core/sd_functions.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <cmath>

static const Rtl433PresetDef rtl433_presets[] = {
    // name                default_freq  mod  dev       rx_bw   d_rate   desc
    {"OOK 433.92M",        433.92f,      2,   0.0f,     270.8f, 10.0f,   "Weather & Security OOK"},
    {"2-FSK 433 17.2k",    433.92f,      0,   19.04f,   135.4f, 17.24f,  "Fine Offset / Ambient FSK"},
    {"2-FSK 433 19.2k",    433.92f,      0,   47.60f,   200.0f, 19.20f,  "Toyota TPMS FSK"},
    {"GFSK 433 17.2k",     433.92f,      1,   19.04f,   135.4f, 17.24f,  "Bresser / Weather GFSK"},
    {"MSK 433 100k",       433.92f,      4,   0.0f,     270.8f, 100.0f,  "433M MSK / Telemetry"},
    {"OOK 868.35M",        868.35f,      2,   0.0f,     270.8f, 10.0f,   "EU Weather OOK"},
    {"2-FSK 868 17.2k",    868.35f,      0,   19.04f,   135.4f, 17.24f,  "EU Fine Offset / LaCrosse"},
    {"GFSK 868 17.2k",     868.35f,      1,   19.04f,   135.4f, 17.24f,  "EU Bresser 5/6/7-in-1 Weather"},
    {"MSK 868 wM-Bus T",   868.95f,      4,   0.0f,     270.8f, 100.0f,  "wM-Bus Mode T (Meters)"},
    {"MSK 868 wM-Bus S",   868.30f,      4,   0.0f,     135.4f, 32.768f, "wM-Bus Mode S (Meters)"},
    {"OOK 345.00M",        345.00f,      2,   0.0f,     270.8f, 10.0f,   "Honeywell / Ademco 5800"},
    {"OOK 315.00M",        315.00f,      2,   0.0f,     270.8f, 10.0f,   "US TPMS & Security OOK"},
    {"2-FSK 315 19.2k",    315.00f,      0,   47.60f,   200.0f, 19.20f,  "US Toyota TPMS FSK"},
    {"GFSK 315 19.2k",     315.00f,      1,   47.60f,   200.0f, 19.20f,  "US TPMS GFSK"},
};

const Rtl433PresetDef *rtl433_get_preset_def(int preset) {
    if (preset < 0 || preset >= RTL433_PRESET_COUNT) preset = RTL433_PRESET_OOK_433;
    return &rtl433_presets[preset];
}

const char *rtl433_get_preset_name(int preset) {
    return rtl433_get_preset_def(preset)->name;
}

std::vector<int> rtl433_get_changing_presets(int changingPreset) {
    switch (changingPreset) {
        case RTL433_CHANGING_433_ALL:
            return {RTL433_PRESET_OOK_433, RTL433_PRESET_FSK_433_17K, RTL433_PRESET_FSK_433_19K, RTL433_PRESET_GFSK_433_17K, RTL433_PRESET_MSK_433_100K};
        case RTL433_CHANGING_868_ALL:
            return {RTL433_PRESET_OOK_868, RTL433_PRESET_FSK_868_17K, RTL433_PRESET_GFSK_868_17K, RTL433_PRESET_MSK_868_T, RTL433_PRESET_MSK_868_S};
        case RTL433_CHANGING_ALL_PRESETS:
            return {
                RTL433_PRESET_OOK_433, RTL433_PRESET_FSK_433_17K, RTL433_PRESET_FSK_433_19K, RTL433_PRESET_GFSK_433_17K, RTL433_PRESET_MSK_433_100K,
                RTL433_PRESET_OOK_868, RTL433_PRESET_FSK_868_17K, RTL433_PRESET_GFSK_868_17K, RTL433_PRESET_MSK_868_T, RTL433_PRESET_MSK_868_S,
                RTL433_PRESET_OOK_345, RTL433_PRESET_OOK_315, RTL433_PRESET_FSK_315_19K, RTL433_PRESET_GFSK_315_19K
            };
        case RTL433_CHANGING_WEATHER:
            return {RTL433_PRESET_OOK_433, RTL433_PRESET_FSK_433_17K, RTL433_PRESET_GFSK_433_17K, RTL433_PRESET_OOK_868, RTL433_PRESET_FSK_868_17K, RTL433_PRESET_GFSK_868_17K};
        case RTL433_CHANGING_TPMS:
            return {RTL433_PRESET_OOK_433, RTL433_PRESET_FSK_433_19K, RTL433_PRESET_OOK_315, RTL433_PRESET_FSK_315_19K, RTL433_PRESET_GFSK_315_19K};
        case RTL433_CHANGING_METERS:
            return {RTL433_PRESET_MSK_868_T, RTL433_PRESET_MSK_868_S, RTL433_PRESET_MSK_433_100K};
        case RTL433_CHANGING_315_ALL:
            return {RTL433_PRESET_OOK_315, RTL433_PRESET_FSK_315_19K, RTL433_PRESET_GFSK_315_19K};
        default:
            return {RTL433_PRESET_OOK_433, RTL433_PRESET_FSK_433_17K, RTL433_PRESET_GFSK_433_17K};
    }
}

const char *rtl433_get_changing_preset_name(int changingPreset) {
    switch (changingPreset) {
        case RTL433_CHANGING_433_ALL: return "433M (All Modes)";
        case RTL433_CHANGING_868_ALL: return "868M (All Modes)";
        case RTL433_CHANGING_ALL_PRESETS: return "All Presets";
        case RTL433_CHANGING_WEATHER: return "Weather Sensors";
        case RTL433_CHANGING_TPMS: return "TPMS Sensors";
        case RTL433_CHANGING_METERS: return "Smart Meters / wM-Bus";
        case RTL433_CHANGING_315_ALL: return "315M (All Modes)";
        default: return "433M (All Modes)";
    }
}

String Rtl433Engine::getActivePresetName() const {
    if (isChangingPreset) {
        return String("Changing: ") + rtl433_get_changing_preset_name(changingPreset);
    }
    return String("Fixed: ") + rtl433_get_preset_name(currentPreset);
}

// ---------------------------------------------------------------------------
// Rtl433Reading formatting
// ---------------------------------------------------------------------------
String Rtl433Reading::toJson() const {
    String json = "{";
    json += "\"time_ms\":" + String(timestamp_ms) + ",";
    json += "\"protocol\":\"" + protocol + "\",";
    json += "\"model\":\"" + model + "\",";
    json += "\"decoder\":\"" + decoder_name + "\",";
    json += "\"frequency\":" + String(frequency, 4) + ",";
    json += "\"modulation\":\"" + modulation + "\",";
    json += "\"rssi\":" + String(rssi) + ",";
    json += "\"id\":" + String(device_id) + ",";

    if (channel >= 0) json += "\"channel\":" + String(channel) + ",";
    if (has_battery) json += "\"battery_ok\":" + String(battery_ok ? "true" : "false") + ",";
    if (has_temp) {
        json += "\"temp_c\":" + String(temp_c, 1) + ",";
        json += "\"temp_f\":" + String(temp_f, 1) + ",";
    }
    if (has_humidity) json += "\"humidity\":" + String(humidity, 1) + ",";
    if (has_pressure) {
        json += "\"pressure_kpa\":" + String(pressure_kpa, 1) + ",";
        json += "\"pressure_psi\":" + String(pressure_psi, 1) + ",";
    }
    if (has_wind) {
        json += "\"wind_speed_ms\":" + String(wind_speed_ms, 1) + ",";
        json += "\"wind_gust_ms\":" + String(wind_gust_ms, 1) + ",";
        if (wind_dir_deg >= 0) json += "\"wind_dir_deg\":" + String(wind_dir_deg) + ",";
    }
    if (has_rain) json += "\"rain_mm\":" + String(rain_mm, 1) + ",";
    if (has_uv) {
        json += "\"uv_index\":" + String(uv_index, 1) + ",";
        json += "\"solar_radiation\":" + String(solar_radiation, 1) + ",";
    }
    if (has_status && status_str.length() > 0) {
        json += "\"status\":\"" + status_str + "\",";
    }

    json += "\"bits\":" + String(bit_len) + ",";
    json += "\"payload\":\"" + payload_hex + "\"";

    if (!raw_durations.empty()) {
        json += ",\"pulses\":[";
        for (size_t i = 0; i < raw_durations.size(); i++) {
            json += String(raw_durations[i]);
            if (i + 1 < raw_durations.size()) json += ",";
        }
        json += "]";
    }

    json += "}";
    return json;
}

String Rtl433Reading::toSummaryLine() const {
    String out = "[" + decoder_name + "] ID:" + String(device_id);
    if (channel >= 0) out += " Ch:" + String(channel);
    if (has_temp) out += " " + String(temp_c, 1) + "C";
    if (has_humidity) out += " " + String((int)humidity) + "%";
    if (has_pressure) out += " " + String(pressure_psi, 1) + "psi";
    if (has_status && status_str.length() > 0) out += " " + status_str;
    return out;
}

String Rtl433Reading::toDetailedText() const {
    String txt = "";
    txt += "Protocol: " + protocol + "\n";
    txt += "Model:    " + model + "\n";
    txt += "Device ID:" + String(device_id);
    if (channel >= 0) txt += "  Ch:" + String(channel);
    txt += "\n";
    if (has_temp) txt += "Temp:     " + String(temp_c, 1) + " C (" + String(temp_f, 1) + " F)\n";
    if (has_humidity) txt += "Humidity: " + String(humidity, 1) + " %\n";
    if (has_pressure) txt += "Pressure: " + String(pressure_kpa, 1) + " kPa (" + String(pressure_psi, 1) + " psi)\n";
    if (has_battery) txt += "Battery:  " + String(battery_ok ? "OK" : "LOW") + "\n";
    if (has_wind) {
        txt += "Wind:     " + String(wind_speed_ms, 1) + " m/s (Gust: " + String(wind_gust_ms, 1) + " m/s";
        if (wind_dir_deg >= 0) txt += " @" + String(wind_dir_deg) + " deg";
        txt += ")\n";
    }
    if (has_rain) txt += "Rain:     " + String(rain_mm, 1) + " mm\n";
    if (has_uv) txt += "UV Index: " + String(uv_index, 1) + "  Solar: " + String(solar_radiation, 0) + " W/m2\n";
    if (has_status && status_str.length() > 0) txt += "Status:   " + status_str + "\n";
    txt += "Freq:     " + String(frequency, 2) + " MHz (" + modulation + ") RSSI:" + String(rssi) + "dBm\n";
    txt += "Payload:  " + payload_hex + " (" + String(bit_len) + " bits)\n";
    txt += "Pulses:   " + String(raw_durations.size()) + " transitions";
    return txt;
}

// ---------------------------------------------------------------------------
// Pulse Demodulators
// ---------------------------------------------------------------------------
static inline bool match_range(int val, int target, int tol_pct) {
    int margin = (target * tol_pct) / 100;
    if (margin < 60) margin = 60;
    return val >= (target - margin) && val <= (target + margin);
}

bool demod_ppm(const std::vector<int> &durations, int mark_us, int zero_gap_us, int one_gap_us, int tol_pct, BitBuffer &out) {
    out.clear();
    if (durations.size() < 10) return false;

    int eff_tol = tol_pct + 25;
    int gap_threshold = (zero_gap_us + one_gap_us) / 2;
    bool zero_is_shorter = zero_gap_us < one_gap_us;

    BitBuffer best_buf;
    BitBuffer cur_buf;

    size_t i = 0;
    while (i + 1 < durations.size()) {
        if (durations[i] <= 0) {
            i++;
            continue;
        }
        int mark = durations[i];
        int gap = -durations[i + 1];

        if (gap <= 0) {
            i++;
            continue;
        }

        if (match_range(mark, mark_us, eff_tol)) {
            bool is_zero = match_range(gap, zero_gap_us, eff_tol) ||
                           (zero_is_shorter ? (gap < gap_threshold && match_range(gap, zero_gap_us, eff_tol + 20))
                                            : (gap >= gap_threshold && match_range(gap, zero_gap_us, eff_tol + 20)));
            bool is_one = match_range(gap, one_gap_us, eff_tol) ||
                          (zero_is_shorter ? (gap >= gap_threshold && match_range(gap, one_gap_us, eff_tol + 20))
                                           : (gap < gap_threshold && match_range(gap, one_gap_us, eff_tol + 20)));

            if (is_zero && !is_one) {
                cur_buf.push_bit(0);
                i += 2;
                continue;
            } else if (is_one && !is_zero) {
                cur_buf.push_bit(1);
                i += 2;
                continue;
            } else if (is_zero && is_one) {
                int diff_zero = abs(gap - zero_gap_us);
                int diff_one = abs(gap - one_gap_us);
                cur_buf.push_bit(diff_zero <= diff_one ? 0 : 1);
                i += 2;
                continue;
            }
        }

        if (cur_buf.num_bits > best_buf.num_bits) {
            best_buf = cur_buf;
        }
        cur_buf.clear();
        i++;
    }
    if (cur_buf.num_bits > best_buf.num_bits) {
        best_buf = cur_buf;
    }
    out = best_buf;
    return out.num_bits >= 16;
}

bool demod_pwm(const std::vector<int> &durations, int zero_mark_us, int one_mark_us, int space_us, int tol_pct, BitBuffer &out) {
    out.clear();
    if (durations.size() < 10) return false;

    int eff_tol = tol_pct + 25;
    int mark_threshold = (zero_mark_us + one_mark_us) / 2;
    bool zero_is_shorter = zero_mark_us < one_mark_us;

    BitBuffer best_buf;
    BitBuffer cur_buf;

    size_t i = 0;
    while (i + 1 < durations.size()) {
        if (durations[i] <= 0) {
            i++;
            continue;
        }
        int mark = durations[i];
        int space = -durations[i + 1];

        if (space <= 0) {
            i++;
            continue;
        }

        bool space_ok = match_range(space, space_us, eff_tol + 15) ||
                        (space >= (space_us - (space_us * eff_tol) / 100) && (space > 5000 || i + 2 >= durations.size()));

        if (space_ok) {
            bool is_zero = match_range(mark, zero_mark_us, eff_tol) ||
                           (zero_is_shorter ? (mark < mark_threshold && match_range(mark, zero_mark_us, eff_tol + 20))
                                            : (mark >= mark_threshold && match_range(mark, zero_mark_us, eff_tol + 20)));
            bool is_one = match_range(mark, one_mark_us, eff_tol) ||
                          (zero_is_shorter ? (mark >= mark_threshold && match_range(mark, one_mark_us, eff_tol + 20))
                                           : (mark < mark_threshold && match_range(mark, one_mark_us, eff_tol + 20)));

            if (is_zero && !is_one) {
                cur_buf.push_bit(0);
                i += 2;
                continue;
            } else if (is_one && !is_zero) {
                cur_buf.push_bit(1);
                i += 2;
                continue;
            } else if (is_zero && is_one) {
                int diff_zero = abs(mark - zero_mark_us);
                int diff_one = abs(mark - one_mark_us);
                cur_buf.push_bit(diff_zero <= diff_one ? 0 : 1);
                i += 2;
                continue;
            }
        }

        if (cur_buf.num_bits > best_buf.num_bits) {
            best_buf = cur_buf;
        }
        cur_buf.clear();
        i++;
    }
    if (cur_buf.num_bits > best_buf.num_bits) {
        best_buf = cur_buf;
    }
    out = best_buf;
    return out.num_bits >= 16;
}

bool demod_pwm_space(const std::vector<int> &durations, int mark_us, int zero_space_us, int one_space_us, int tol_pct, BitBuffer &out) {
    out.clear();
    if (durations.size() < 10) return false;

    int eff_tol = tol_pct + 25;
    int space_threshold = (zero_space_us + one_space_us) / 2;
    bool zero_is_shorter = zero_space_us < one_space_us;

    BitBuffer best_buf;
    BitBuffer cur_buf;

    size_t i = 0;
    while (i + 1 < durations.size()) {
        if (durations[i] <= 0) {
            i++;
            continue;
        }
        int mark = durations[i];
        int space = -durations[i + 1];

        if (space <= 0) {
            i++;
            continue;
        }

        if (match_range(mark, mark_us, eff_tol)) {
            bool is_zero = match_range(space, zero_space_us, eff_tol) ||
                           (zero_is_shorter ? (space < space_threshold && match_range(space, zero_space_us, eff_tol + 20))
                                            : (space >= space_threshold && match_range(space, zero_space_us, eff_tol + 20)));
            bool is_one = match_range(space, one_space_us, eff_tol) ||
                          (zero_is_shorter ? (space >= space_threshold && match_range(space, one_space_us, eff_tol + 20))
                                           : (space < space_threshold && match_range(space, one_space_us, eff_tol + 20)));

            if (is_zero && !is_one) {
                cur_buf.push_bit(0);
                i += 2;
                continue;
            } else if (is_one && !is_zero) {
                cur_buf.push_bit(1);
                i += 2;
                continue;
            } else if (is_zero && is_one) {
                int diff_zero = abs(space - zero_space_us);
                int diff_one = abs(space - one_space_us);
                cur_buf.push_bit(diff_zero <= diff_one ? 0 : 1);
                i += 2;
                continue;
            }
        }

        if (cur_buf.num_bits > best_buf.num_bits) {
            best_buf = cur_buf;
        }
        cur_buf.clear();
        i++;
    }
    if (cur_buf.num_bits > best_buf.num_bits) {
        best_buf = cur_buf;
    }
    out = best_buf;
    return out.num_bits >= 16;
}

bool demod_manchester(const std::vector<int> &durations, int half_clock_us, int tol_pct, BitBuffer &out, bool invert) {
    out.clear();
    if (durations.size() < 8) return false;

    int clock_us = half_clock_us * 2;
    int half_min = (half_clock_us * 30) / 100;
    if (half_min < 35) half_min = 35;
    int half_max = (half_clock_us * 170) / 100;

    int full_min = (clock_us * 60) / 100;
    int full_max = (clock_us * 150) / 100;

    BitBuffer best_buf;
    BitBuffer cur_buf;
    int state = 0; // 0 = start/sync, 1 = bit middle

    for (int d : durations) {
        int len = abs(d);
        int level = (d > 0) ? 1 : 0;

        bool is_half = (len >= half_min && len <= half_max);
        bool is_full = (len >= full_min && len <= full_max);

        if (!is_half && !is_full) {
            if (cur_buf.num_bits > best_buf.num_bits) {
                best_buf = cur_buf;
            }
            cur_buf.clear();
            state = 0;
            continue;
        }

        if (state == 0) {
            state = 1;
        } else {
            if (is_half) {
                cur_buf.push_bit(invert ? !level : level);
                state = 0;
            } else if (is_full) {
                cur_buf.push_bit(invert ? !level : level);
                state = 1;
            }
        }
    }
    if (cur_buf.num_bits > best_buf.num_bits) {
        best_buf = cur_buf;
    }
    out = best_buf;
    return out.num_bits >= 16;
}

bool demod_pcm_fsk(const std::vector<int> &durations, int bit_period_us, int tol_pct, BitBuffer &out, uint32_t sync_word, uint8_t sync_len) {
    out.clear();
    if (durations.size() < 6 || bit_period_us <= 0) return false;

    int min_period = (bit_period_us * (100 - tol_pct)) / 100;
    if (min_period < 4) min_period = 4;

    BitBuffer best_buf;
    BitBuffer cur_buf;
    uint32_t shift_reg = 0;
    uint32_t sync_mask = (sync_len >= 32) ? 0xFFFFFFFF : ((1UL << sync_len) - 1);
    bool capturing_synced = false;

    for (int d : durations) {
        int len = abs(d);
        int bit_val = (d > 0) ? 1 : 0;

        if (len < min_period / 2) continue; // ignore tiny glitch

        int count = (len + bit_period_us / 2) / bit_period_us;
        if (count == 0) count = 1;
        if (count > 128) {
            if (cur_buf.num_bits > best_buf.num_bits) {
                best_buf = cur_buf;
            }
            cur_buf.clear();
            shift_reg = 0;
            capturing_synced = false;
            continue;
        }

        for (int i = 0; i < count; i++) {
            if (sync_len > 0) {
                shift_reg = (shift_reg << 1) | bit_val;
                if (!capturing_synced) {
                    if ((shift_reg & sync_mask) == (sync_word & sync_mask)) {
                        capturing_synced = true;
                        cur_buf.clear();
                        for (int s = (int)sync_len - 1; s >= 0; s--) {
                            cur_buf.push_bit((sync_word >> s) & 1);
                        }
                    }
                } else {
                    cur_buf.push_bit(bit_val);
                    if (cur_buf.num_bits >= BitBuffer::MAX_BYTES * 8) {
                        break;
                    }
                }
            } else {
                cur_buf.push_bit(bit_val);
            }
        }
    }
    if (cur_buf.num_bits > best_buf.num_bits) {
        best_buf = cur_buf;
    }
    out = best_buf;
    return out.num_bits >= 16;
}

// ---------------------------------------------------------------------------
// Engine Implementation
// ---------------------------------------------------------------------------
Rtl433Engine &Rtl433Engine::instance() {
    static Rtl433Engine inst;
    return inst;
}

bool Rtl433Engine::initRadio(float freq, int preset) {
    currentPreset = preset;
    currentFrequency = (freq > 0.0f) ? freq : rtl433_get_preset_def(preset)->default_freq;

    const Rtl433PresetDef *pdef = rtl433_get_preset_def(currentPreset);

    // initRfModule() now applies the correct modulation-specific register/AGC preset (OOK vs
    // FSK-family) directly, so no post-hoc patching of modulation/deviation/bandwidth/data-rate
    // is needed here anymore.
    if (!initRfModule("rx", currentFrequency, pdef->modulation, pdef->deviation, pdef->rx_bw, pdef->data_rate))
        return false;

    if (bruceConfigPins.rfModule == CC1101_SPI_MODULE) {
        tft.drawPixel(0, 0, 0); // Keep shared SPI bus clean for display
    }
    return true;
}

bool Rtl433Engine::switchPreset(float freq, int preset) {
    currentPreset = preset;
    currentFrequency = (freq > 0.0f) ? freq : rtl433_get_preset_def(preset)->default_freq;

    const Rtl433PresetDef *pdef = rtl433_get_preset_def(currentPreset);

    if (bruceConfigPins.rfModule == CC1101_SPI_MODULE) {
        ELECHOUSE_cc1101.setSidle();
        setMHZ(currentFrequency);
        ELECHOUSE_cc1101.setModulation(pdef->modulation);
        // Re-apply the correct fixed-frequency register/AGC preset for the new modulation
        // (radio is already running, so we can't go through a full initRfModule() re-init here).
        if (pdef->modulation == 2) {
            cc1101ApplyFixedFreqOokPreset(false);
        } else {
            cc1101ApplyFixedFreqFskPreset(false);
        }
        if (pdef->modulation != 2) {
            if (pdef->deviation > 0.0f) ELECHOUSE_cc1101.setDeviation(pdef->deviation);
            if (pdef->rx_bw > 0.0f) ELECHOUSE_cc1101.setRxBW(pdef->rx_bw);
            if (pdef->data_rate > 0.0f) ELECHOUSE_cc1101.setDRate(pdef->data_rate);
            ELECHOUSE_cc1101.setSyncMode(0); // Unfiltered continuous async slicer stream
            ELECHOUSE_cc1101.setDcFilterOff(true);
        } else if (pdef->rx_bw > 0.0f) {
            ELECHOUSE_cc1101.setRxBW(pdef->rx_bw);
        }
        ELECHOUSE_cc1101.setPktFormat(3); // Asynchronous serial mode
        pinMode(bruceConfigPins.CC1101_bus.io0, INPUT);
        ELECHOUSE_cc1101.SetRx();
        // GDO0 = 0x0D (Serial Data Output, async) for all modulations. 0x0E is "Carrier
        // sense" per the CC1101 datasheet, not a data-output mode - it carries no demodulated
        // bit data at all, which is why FSK/GFSK/MSK presets never produced usable captures.
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x0D);
        tft.drawPixel(0, 0, 0); // Keep shared SPI bus clean for display
    } else {
        bruceConfigPins.setRfFreq(currentFrequency, 1);
    }
    return true;
}

void Rtl433Engine::deinitRadio() {
    deinitRfModule();
}

bool Rtl433Engine::decode(const std::vector<int> &durations, float freq, int preset, int rssi, Rtl433Reading &reading) {
    if (durations.size() < 10) return false;
    _packetsReceived++;

    const Rtl433PresetDef *pdef = rtl433_get_preset_def(preset);
    int mod = pdef->modulation;

    bool ok = false;
    String modStr = "OOK";

    if (mod == 4) {
        // MSK Decoders
        modStr = "MSK";
        ok = decode_wmbus(durations, reading);
    } else if (mod == 0 || mod == 1) {
        // 2-FSK & GFSK Decoders
        modStr = (mod == 1) ? "GFSK" : "2-FSK";
        ok = decode_fineoffset_fsk(durations, reading) ||
             decode_bresser_5in1(durations, reading) ||
             decode_bresser_6in1(durations, reading) ||
             decode_toyota_tpms(durations, reading) ||
             decode_lacrosse_tx(durations, reading) ||
             decode_wmbus(durations, reading);
    } else {
        // OOK Decoders
        modStr = "OOK";
        ok = decode_nexus(durations, reading) ||
             decode_acurite_606tx(durations, reading) ||
             decode_acurite_tower(durations, reading) ||
             decode_oregon_scientific(durations, reading) ||
             decode_honeywell_5800(durations, reading) ||
             decode_schrader_tpms(durations, reading) ||
             decode_kerui_ev1527(durations, reading) ||
             decode_dsc_security(durations, reading) ||
             decode_proove_nexa(durations, reading) ||
             decode_lacrosse_tx(durations, reading);
    }

    if (ok) {
        _packetsDecoded++;
        reading.frequency = freq;
        reading.modulation = modStr;
        reading.preset_idx = preset;
        reading.rssi = rssi;
        reading.raw_durations = durations;
        reading.timestamp_ms = millis();
        return true;
    }
    return false;
}

void Rtl433Engine::addRecent(const Rtl433Reading &reading) {
    if (_recentReadings.size() >= MAX_RECENT) {
        _recentReadings.erase(_recentReadings.begin());
    }
    _recentReadings.push_back(reading);
}

const Rtl433Reading *Rtl433Engine::getRecentAt(size_t index) const {
    if (index >= _recentReadings.size()) return nullptr;
    return &_recentReadings[index];
}

bool Rtl433Engine::logJson(const Rtl433Reading &reading, bool sd_enabled) {
    String line = reading.toJson() + "\n";
    Serial.print(line); // Stream to serial interface

    if (!sd_enabled) return true;

    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return false;

    // Create /rtl433 directory if not existing
    if (!fs->exists("/rtl433")) {
        fs->mkdir("/rtl433");
    }

    File logFile = fs->open("/rtl433/traffic.json", FILE_APPEND);
    if (!logFile) {
        logFile = fs->open("/rtl433_traffic.json", FILE_APPEND);
    }
    if (!logFile) return false;

    logFile.print(line);
    logFile.close();
    return true;
}

bool Rtl433Engine::replayReading(const Rtl433Reading &reading, int repeatCount) {
    if (reading.raw_durations.empty()) return false;

    float freqMhz = reading.frequency;
    if (freqMhz < 280.0f || freqMhz > 928.0f) freqMhz = bruceConfigPins.rfFreq;

    int presetIdx = reading.preset_idx;
    if (presetIdx < 0 || presetIdx >= RTL433_PRESET_COUNT || (reading.modulation != "" && reading.modulation != "OOK" && presetIdx == RTL433_PRESET_OOK_433)) {
        if (reading.modulation == "MSK") {
            presetIdx = (freqMhz > 800.0f) ? RTL433_PRESET_MSK_868_T : RTL433_PRESET_MSK_433_100K;
        } else if (reading.modulation == "GFSK") {
            presetIdx = (freqMhz > 800.0f) ? RTL433_PRESET_GFSK_868_17K :
                        (freqMhz < 330.0f) ? RTL433_PRESET_GFSK_315_19K : RTL433_PRESET_GFSK_433_17K;
        } else if (reading.modulation == "2-FSK" || reading.modulation == "FSK") {
            presetIdx = (freqMhz > 800.0f) ? RTL433_PRESET_FSK_868_17K :
                        (freqMhz < 330.0f) ? RTL433_PRESET_FSK_315_19K : RTL433_PRESET_FSK_433_17K;
        } else {
            presetIdx = (freqMhz > 800.0f) ? RTL433_PRESET_OOK_868 :
                        (freqMhz < 330.0f) ? RTL433_PRESET_OOK_315 :
                        (freqMhz > 330.0f && freqMhz < 360.0f) ? RTL433_PRESET_OOK_345 : RTL433_PRESET_OOK_433;
        }
    }
    const Rtl433PresetDef *pdef = rtl433_get_preset_def(presetIdx);

    int repeats = (repeatCount < 1) ? replayRepeats : repeatCount;
    if (repeats < 1) repeats = 3;

    int gap_us = (replayGapMs > 0) ? -(replayGapMs * 1000) : -20000;

    // Synthesize preamble (lead-in training sequence for receiver bit-sync & AGC / discriminator lock)
    std::vector<int> preambleDurs;
    if (replayPreamble || pdef->modulation != 2 || reading.modulation == "2-FSK" || reading.modulation == "GFSK" || reading.modulation == "MSK") {
        int bit_us = 58;
        if (reading.decoder_id == 13) {
            bit_us = 122; // Bresser 5-in-1 standard ~8.21 kbps
        } else if (reading.decoder_id == 14) {
            bit_us = 125; // Bresser 6-in-1 standard ~8.0 kbps
        } else if (reading.decoder_id == 5) {
            bit_us = 58;  // FineOffset standard ~17.24 kbps
        } else if (reading.decoder_id == 15 || reading.decoder_id == 17) {
            bit_us = 10;  // wM-Bus Mode T 100 kbps
        } else if (pdef->data_rate > 0.0f) {
            bit_us = (int)(1000.0f / pdef->data_rate + 0.5f);
        } else if (pdef->modulation == 2) { // OOK
            bit_us = 500;
        }
        if (bit_us < 8) bit_us = 8;
        if (bit_us > 2000) bit_us = 2000;

        // Generate 32 alternating bits (16 mark/space cycles: 0xAA 0xAA 0xAA 0xAA)
        for (int p = 0; p < 16; p++) {
            preambleDurs.push_back(bit_us);
            preambleDurs.push_back(-bit_us);
        }
    }

    std::vector<int> transmitDurs;
    transmitDurs.reserve(preambleDurs.size() * repeats + reading.raw_durations.size() * repeats + repeats);
    for (int r = 0; r < repeats; r++) {
        if (!preambleDurs.empty()) {
            transmitDurs.insert(transmitDurs.end(), preambleDurs.begin(), preambleDurs.end());
        }
        for (int d : reading.raw_durations) {
            transmitDurs.push_back(d);
        }
        // Ensure silence gap between packet frames
        if (transmitDurs.empty() || transmitDurs.back() > 0) {
            transmitDurs.push_back(gap_us);
        } else if (transmitDurs.back() > gap_us) {
            transmitDurs.back() = gap_us;
        }
    }

    // Build target frequencies list based on spread option to account for crystal/receiver offset
    std::vector<float> targetFreqs;
    if (replayFreqSpread == 1) { // +/- 15 kHz (3 frequencies)
        targetFreqs = { freqMhz - 0.015f, freqMhz, freqMhz + 0.015f };
    } else if (replayFreqSpread == 2) { // +/- 30 kHz (5 frequencies)
        targetFreqs = { freqMhz - 0.030f, freqMhz - 0.015f, freqMhz, freqMhz + 0.015f, freqMhz + 0.030f };
    } else if (replayFreqSpread == 3) { // +/- 50 kHz (3 frequencies)
        targetFreqs = { freqMhz - 0.050f, freqMhz, freqMhz + 0.050f };
    } else {
        targetFreqs = { freqMhz };
    }

    for (size_t f_idx = 0; f_idx < targetFreqs.size(); f_idx++) {
        float curFreq = targetFreqs[f_idx];
        // initRfModule() now applies the correct modulation-specific register/AGC preset (OOK vs
        // FSK-family) directly, so no post-hoc patching of modulation/deviation/data-rate is
        // needed here anymore.
        if (!initRfModule("tx", curFreq, pdef->modulation, pdef->deviation, 0.0f, pdef->data_rate)) continue;

        if (bruceConfigPins.rfModule == CC1101_SPI_MODULE) {
            pinMode(bruceConfigPins.CC1101_bus.io0, OUTPUT);
            ELECHOUSE_cc1101.setPA(bruceConfigPins.rfTxPower);
            ioExpander.turnPinOnOff(IO_EXP_CC_RX, LOW);
            ioExpander.turnPinOnOff(IO_EXP_CC_TX, HIGH);
            ELECHOUSE_cc1101.SetTx();
            ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x2E);
            delayMicroseconds(500); // Allow PLL lock and PA ramp-up to settle
        }

        rf_tx_durations(transmitDurs);
        deinitRfModule();
        if (f_idx + 1 < targetFreqs.size()) {
            delay(30);
        }
    }

    return true;
}

bool Rtl433Engine::saveSubFile(const Rtl433Reading &reading, String *outFilename) {
    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return false;

    if (!fs->exists("/BruceRF")) fs->mkdir("/BruceRF");

    String cleanName = reading.decoder_name.length() > 0 ? reading.decoder_name : "RTL433";
    cleanName.replace(" ", "_");
    cleanName.replace("/", "_");

    String cleanMod = reading.modulation.length() > 0 ? reading.modulation : "OOK";
    cleanMod.replace(" ", "_");
    cleanMod.replace("/", "_");
    cleanMod.replace("-", "");

    char freqBuf[16];
    snprintf(freqBuf, sizeof(freqBuf), "%.2fM", reading.frequency);

    String base = "/BruceRF/" + cleanName + "_" + String(freqBuf) + "_" + cleanMod + "_" + String(reading.device_id);
    String path = base + ".sub";
    int idx = 1;
    while (fs->exists(path)) {
        path = base + "_" + String(idx++) + ".sub";
    }

    File file = fs->open(path, FILE_WRITE);
    if (!file) return false;

    file.println("Filetype: Bruce SubGhz File");
    file.println("Version 1");
    file.println("Frequency: " + String((int)(reading.frequency * 1000000)));
    file.println("Preset: " + String((reading.modulation == "2-FSK" || reading.modulation == "FSK" || reading.modulation == "GFSK" || reading.modulation == "MSK") ? "2FSKDev238Async" : "Ook270Async"));
    file.println("Protocol: RAW");

    String rawStr = "RAW_Data: ";
    for (size_t i = 0; i < reading.raw_durations.size(); i++) {
        rawStr += String(reading.raw_durations[i]) + " ";
        if (rawStr.length() > 180) {
            file.println(rawStr);
            rawStr = "RAW_Data: ";
        }
    }
    if (rawStr.length() > 10) file.println(rawStr);

    file.close();
    if (outFilename) *outFilename = path;
    return true;
}

size_t Rtl433Engine::saveAllSubFiles(int *savedCount) {
    int saved = 0;
    for (const auto &reading : _recentReadings) {
        if (saveSubFile(reading)) {
            saved++;
        }
    }
    if (savedCount) *savedCount = saved;
    return saved;
}

// ---------------------------------------------------------------------------
// Unit Self-Test Implementation
// ---------------------------------------------------------------------------
static std::vector<int> build_ppm_pulses(const uint8_t *bytes, size_t bit_count, int mark_us, int zero_gap_us, int one_gap_us) {
    std::vector<int> durs;
    durs.push_back(mark_us);
    durs.push_back(-4000); // sync gap
    for (size_t i = 0; i < bit_count; i++) {
        uint8_t byte_val = bytes[i / 8];
        uint8_t bit = (byte_val >> (7 - (i % 8))) & 1;
        durs.push_back(mark_us);
        durs.push_back(bit ? -one_gap_us : -zero_gap_us);
    }
    durs.push_back(mark_us); // Stop mark to delimit final bit gap
    return durs;
}

static std::vector<int> build_pwm_pulses(const uint8_t *bytes, size_t bit_count, int zero_mark_us, int one_mark_us, int space_us) {
    std::vector<int> durs;
    for (size_t i = 0; i < bit_count; i++) {
        uint8_t byte_val = bytes[i / 8];
        uint8_t bit = (byte_val >> (7 - (i % 8))) & 1;
        durs.push_back(bit ? one_mark_us : zero_mark_us);
        durs.push_back(-space_us);
    }
    return durs;
}

static std::vector<int> build_manchester_pulses(const uint8_t *bytes, size_t bit_count, int half_us) {
    std::vector<int> durs;
    int current_level = 1;
    int current_len = 0;

    auto push_level = [&](int level) {
        if (current_len == 0) {
            current_level = level;
            current_len = half_us;
        } else if (current_level == level) {
            current_len += half_us;
        } else {
            durs.push_back(current_level ? current_len : -current_len);
            current_level = level;
            current_len = half_us;
        }
    };

    for (size_t i = 0; i < bit_count; i++) {
        uint8_t byte_val = bytes[i / 8];
        uint8_t bit = (byte_val >> (7 - (i % 8))) & 1;
        if (bit) {
            push_level(0);
            push_level(1);
        } else {
            push_level(1);
            push_level(0);
        }
    }
    if (current_len > 0) durs.push_back(current_level ? current_len : -current_len);
    return durs;
}

static std::vector<int> build_pcm_pulses(const uint8_t *bytes, size_t bit_count, int bit_us) {
    std::vector<int> durs;
    int current_level = (bytes[0] >> 7) & 1;
    int current_len = 0;

    for (size_t i = 0; i < bit_count; i++) {
        uint8_t byte_val = bytes[i / 8];
        uint8_t bit = (byte_val >> (7 - (i % 8))) & 1;
        if (current_len == 0) {
            current_level = bit;
            current_len = bit_us;
        } else if (current_level == bit) {
            current_len += bit_us;
        } else {
            durs.push_back(current_level ? current_len : -current_len);
            current_level = bit;
            current_len = bit_us;
        }
    }
    if (current_len > 0) durs.push_back(current_level ? current_len : -current_len);
    return durs;
}

bool rtl433_selftest(String &report) {
    report = "=== RTL433 Decoder Self-Test ===\n";
    int passed = 0;
    int total = 0;

    // Test 1: Nexus (OOK PPM)
    {
        total++;
        // ID=0x8E (142), Channel=1 (b1=0x00), Temp=25.4C (0x00FE=254), Hum=55% (0x37 -> b3=0xF3, b4=0x70)
        uint8_t nexus_data[] = {0x8E, 0x00, 0xFE, 0xF3, 0x70};
        std::vector<int> durs = build_ppm_pulses(nexus_data, 36, 500, 1000, 2000);
        Rtl433Reading r;
        if (decode_nexus(durs, r) && r.device_id == 0x8E && abs(r.temp_c - 25.4f) < 0.2f && abs(r.humidity - 55.0f) < 0.2f) {
            report += "[PASS] Nexus / Rubicson OOK PPM\n";
            passed++;
        } else {
            report += "[FAIL] Nexus / Rubicson OOK PPM (id=" + String(r.device_id) + " temp=" + String(r.temp_c) + " hum=" + String(r.humidity) + ")\n";
        }
    }

    // Test 2: Acurite 606TX (OOK PWM)
    {
        total++;
        // ID=0x55, Ch=1 & TempMSB=4 (0x04), TempLSB=0xB0 (raw 1200 = 20.0C), Checksum = (0x55+0x04+0xB0)&0xFF = 0x09
        uint8_t acurite_data[] = {0x55, 0x04, 0xB0, 0x09};
        std::vector<int> durs = build_pwm_pulses(acurite_data, 32, 200, 600, 400);
        Rtl433Reading r;
        if (decode_acurite_606tx(durs, r) && r.device_id == 0x55 && abs(r.temp_c - 20.0f) < 0.2f) {
            report += "[PASS] Acurite 606TX OOK PWM\n";
            passed++;
        } else {
            report += "[FAIL] Acurite 606TX OOK PWM\n";
        }
    }

    // Test 3: EV1527 Security (OOK PWM)
    {
        total++;
        // Addr=0x12345, Cmd=0x08 (DOOR OPEN / MOTION) -> bytes: 0x12, 0x34, 0x58
        uint8_t ev_data[] = {0x12, 0x34, 0x58};
        std::vector<int> durs;
        for (size_t i = 0; i < 24; i++) {
            uint8_t bit = (ev_data[i / 8] >> (7 - (i % 8))) & 1;
            durs.push_back(350);
            durs.push_back(bit ? -350 : -1050);
        }
        Rtl433Reading r;
        if (decode_kerui_ev1527(durs, r) && r.device_id == 0x12345 && r.status_flags == 0x08) {
            report += "[PASS] Kerui / EV1527 Alarm OOK PWM\n";
            passed++;
        } else {
            report += "[FAIL] Kerui / EV1527 Alarm OOK PWM\n";
        }
    }

    // Test 4: Honeywell 5800 (OOK Manchester)
    {
        total++;
        // Sync + 24-bit ID + status byte + parity
        uint8_t hw_data[] = {0xFF, 0x1A, 0x2B, 0x3C, 0x90, 0x00, 0x12, 0x34};
        std::vector<int> durs = build_manchester_pulses(hw_data, 64, 380);
        Rtl433Reading r;
        if (decode_honeywell_5800(durs, r) && r.device_id == 0x1A2B3C) {
            report += "[PASS] Honeywell 5800 Security OOK Manchester\n";
            passed++;
        } else {
            report += "[FAIL] Honeywell 5800 Security OOK Manchester\n";
        }
    }

    // Test 5: Fine Offset WH65 FSK (2-FSK PCM)
    {
        total++;
        // Preamble + Sync 0x2DD4 + Family 0x48 (WH65B) + ID 0x1234 + Temp 615 (21.5C) + Hum 50%
        BitBuffer b;
        // sync
        for (int i = 15; i >= 0; i--) b.push_bit((0x2DD4 >> i) & 1);
        uint8_t payload[14] = {0x48, 0x12, 0x34, 0x02, 0x67, 50, 90, 15, 25, 0, 50, 3, 100, 0};
        for (int i = 0; i < 13; i++) {
            for (int bit = 7; bit >= 0; bit--) b.push_bit((payload[i] >> bit) & 1);
        }
        uint8_t crc = b.crc8(0x31, 0x00, 16, 13 * 8);
        for (int bit = 7; bit >= 0; bit--) b.push_bit((crc >> bit) & 1);

        std::vector<int> durs = build_pcm_pulses(b.data, b.num_bits, 58);
        Rtl433Reading r;
        if (decode_fineoffset_fsk(durs, r) && r.device_id == 0x1234 && abs(r.temp_c - 21.5f) < 0.2f && r.humidity == 50.0f) {
            report += "[PASS] Fine Offset WH65 Weather 2-FSK\n";
            passed++;
        } else {
            report += "[FAIL] Fine Offset WH65 Weather 2-FSK\n";
        }
    }

    // Test 6: Bresser 5-in-1 (GFSK PCM)
    {
        total++;
        BitBuffer b;
        for (int i = 15; i >= 0; i--) b.push_bit((0x2DD4 >> i) & 1);
        uint8_t payload[10] = {0x51, 0x82, 0x00, 0xDE, 55, 0x04, 25, 0x00, 0x14, 0x00};
        uint8_t sum = 0;
        for (int i = 0; i < 9; i++) sum += payload[i];
        payload[9] = sum;
        for (int i = 0; i < 10; i++) {
            for (int bit = 7; bit >= 0; bit--) b.push_bit((payload[i] >> bit) & 1);
        }
        std::vector<int> durs = build_pcm_pulses(b.data, b.num_bits, 122);
        Rtl433Reading r;
        if (decode_bresser_5in1(durs, r) && (r.device_id == ((0x51 << 8) | 0x02)) && abs(r.temp_c - 22.2f) < 0.2f && r.humidity == 55.0f) {
            report += "[PASS] Bresser 5-in-1 Weather GFSK\n";
            passed++;
        } else {
            report += "[FAIL] Bresser 5-in-1 Weather GFSK (temp=" + String(r.temp_c) + " hum=" + String(r.humidity) + ")\n";
        }
    }

    // Test 7: Wireless M-Bus Mode T (MSK PCM)
    {
        total++;
        BitBuffer b;
        // Sync word 0x543D
        for (int i = 15; i >= 0; i--) b.push_bit((0x543D >> i) & 1);
        // L=0x1E, C=0x44 (SND_NR), Manuf=0x2D2C (KAM), ID=0x78563412 (12345678), Ver=0x01, Type=0x07 (Water)
        uint8_t wmbus_hdr[] = {0x1E, 0x44, 0x2D, 0x2C, 0x78, 0x56, 0x34, 0x12, 0x01, 0x07};
        for (size_t i = 0; i < sizeof(wmbus_hdr); i++) {
            for (int bit = 7; bit >= 0; bit--) b.push_bit((wmbus_hdr[i] >> bit) & 1);
        }
        std::vector<int> durs = build_pcm_pulses(b.data, b.num_bits, 10);
        Rtl433Reading r;
        if (decode_wmbus(durs, r) && r.device_id == 0x12345678 && r.channel == 0x07) {
            report += "[PASS] Wireless M-Bus Mode T MSK (Water Meter 12345678)\n";
            passed++;
        } else {
            report += "[FAIL] Wireless M-Bus Mode T MSK (id=" + String(r.device_id, HEX) + " type=" + String(r.channel) + ")\n";
        }
    }

    report += "Result: " + String(passed) + "/" + String(total) + " tests passed.\n";
    return passed == total;
}

bool Rtl433Engine::transmitSample(const String &sampleType, float freq, int repeats) {
    String st = sampleType;
    st.toLowerCase();
    st.trim();

    Rtl433Reading r;
    r.device_id = 0x1234;
    int presetIdx = RTL433_PRESET_OOK_433;

    if (st == "nexus" || st == "ook" || st == "ppm" || st == "rubicson") {
        uint8_t nexus_data[] = {0x8E, 0x00, 0xFE, 0xF3, 0x70};
        r.raw_durations = build_ppm_pulses(nexus_data, 36, 500, 1000, 2000);
        r.protocol = "Nexus-TH";
        r.model = "Nexus / Rubicson";
        r.decoder_name = "Nexus";
        r.modulation = "OOK";
        r.decoder_id = 1;
        float defFreq = (freq > 0.0f) ? freq : 433.92f;
        r.frequency = defFreq;
        presetIdx = (defFreq > 800.0f) ? RTL433_PRESET_OOK_868 :
                    (defFreq < 330.0f) ? RTL433_PRESET_OOK_315 :
                    (defFreq > 330.0f && defFreq < 360.0f) ? RTL433_PRESET_OOK_345 : RTL433_PRESET_OOK_433;
    } else if (st == "acurite" || st == "pwm" || st == "606tx") {
        uint8_t acurite_data[] = {0x55, 0x04, 0xB0, 0x09};
        r.raw_durations = build_pwm_pulses(acurite_data, 32, 200, 600, 400);
        r.protocol = "Acurite-606TX";
        r.model = "606TX";
        r.decoder_name = "Acurite";
        r.modulation = "OOK";
        r.decoder_id = 2;
        float defFreq = (freq > 0.0f) ? freq : 433.92f;
        r.frequency = defFreq;
        presetIdx = (defFreq > 800.0f) ? RTL433_PRESET_OOK_868 :
                    (defFreq < 330.0f) ? RTL433_PRESET_OOK_315 : RTL433_PRESET_OOK_433;
    } else if (st == "honeywell" || st == "5800" || st == "manchester" || st == "345") {
        uint8_t hw_data[] = {0xFF, 0x1A, 0x2B, 0x3C, 0x90, 0x00, 0x12, 0x34};
        r.raw_durations = build_manchester_pulses(hw_data, 64, 380);
        r.protocol = "Honeywell-5800";
        r.model = "5800 Door/Window";
        r.decoder_name = "Honeywell";
        r.modulation = "OOK";
        r.decoder_id = 7;
        float defFreq = (freq > 0.0f) ? freq : 433.92f;
        r.frequency = defFreq;
        presetIdx = (defFreq > 800.0f) ? RTL433_PRESET_OOK_868 :
                    (defFreq < 330.0f) ? RTL433_PRESET_OOK_315 :
                    (defFreq > 330.0f && defFreq < 360.0f) ? RTL433_PRESET_OOK_345 : RTL433_PRESET_OOK_433;
    } else if (st == "wh65" || st == "fsk" || st == "2fsk" || st == "fineoffset") {
        BitBuffer b;
        for (int i = 15; i >= 0; i--) b.push_bit((0x2DD4 >> i) & 1);
        uint8_t payload[14] = {0x48, 0x12, 0x34, 0x02, 0x67, 50, 90, 15, 25, 0, 50, 3, 100, 0};
        for (int i = 0; i < 13; i++) {
            for (int bit = 7; bit >= 0; bit--) b.push_bit((payload[i] >> bit) & 1);
        }
        uint8_t crc = b.crc8(0x31, 0x00, 16, 13 * 8);
        for (int bit = 7; bit >= 0; bit--) b.push_bit((crc >> bit) & 1);

        r.raw_durations = build_pcm_pulses(b.data, b.num_bits, 58);
        r.protocol = "FineOffset-WH65";
        r.model = "WH65B Station";
        r.decoder_name = "FineOffset";
        r.modulation = "2-FSK";
        r.decoder_id = 5;
        float defFreq = (freq > 0.0f) ? freq : 433.92f;
        r.frequency = defFreq;
        presetIdx = (defFreq > 800.0f) ? RTL433_PRESET_FSK_868_17K :
                    (defFreq < 330.0f) ? RTL433_PRESET_FSK_315_19K : RTL433_PRESET_FSK_433_17K;
    } else if (st == "bresser" || st == "gfsk" || st == "5in1") {
        BitBuffer b;
        for (int i = 15; i >= 0; i--) b.push_bit((0x2DD4 >> i) & 1);
        uint8_t payload[10] = {0x51, 0x82, 0x00, 0xDE, 55, 0x04, 25, 0x00, 0x14, 0x00};
        uint8_t sum = 0;
        for (int i = 0; i < 9; i++) sum += payload[i];
        payload[9] = sum;
        for (int i = 0; i < 10; i++) {
            for (int bit = 7; bit >= 0; bit--) b.push_bit((payload[i] >> bit) & 1);
        }
        r.raw_durations = build_pcm_pulses(b.data, b.num_bits, 122);
        r.protocol = "Bresser-5in1";
        r.model = "5-in-1 Weather";
        r.decoder_name = "Bresser";
        r.modulation = "GFSK";
        r.decoder_id = 13;
        float defFreq = (freq > 0.0f) ? freq : 433.92f;
        r.frequency = defFreq;
        presetIdx = (defFreq > 800.0f) ? RTL433_PRESET_GFSK_868_17K :
                    (defFreq < 330.0f) ? RTL433_PRESET_GFSK_315_19K : RTL433_PRESET_GFSK_433_17K;
    } else if (st == "wmbus" || st == "msk" || st == "mskt" || st == "wmbust") {
        BitBuffer b;
        for (int i = 15; i >= 0; i--) b.push_bit((0x543D >> i) & 1);
        uint8_t wmbus_hdr[] = {0x1E, 0x44, 0x2D, 0x2C, 0x78, 0x56, 0x34, 0x12, 0x01, 0x07};
        for (size_t i = 0; i < sizeof(wmbus_hdr); i++) {
            for (int bit = 7; bit >= 0; bit--) b.push_bit((wmbus_hdr[i] >> bit) & 1);
        }
        r.raw_durations = build_pcm_pulses(b.data, b.num_bits, 10);
        r.protocol = "Wireless-MBus";
        r.model = "wM-Bus Mode T (Water)";
        r.decoder_name = "wM-Bus";
        r.modulation = "MSK";
        r.decoder_id = 15;
        float defFreq = (freq > 0.0f) ? freq : 433.92f;
        r.frequency = defFreq;
        presetIdx = (defFreq > 800.0f) ? RTL433_PRESET_MSK_868_T : RTL433_PRESET_MSK_433_100K;
    } else {
        return false;
    }

    r.preset_idx = presetIdx;
    return replayReading(r, repeats);
}
