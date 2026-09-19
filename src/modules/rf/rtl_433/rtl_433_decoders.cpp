// SPDX-License-Identifier: AGPL-3.0-or-later
#include "rtl_433.h"
#include <cmath>

// ===========================================================================
// Decoder 1: Nexus / Fine Offset / Rubicson / TFA (OOK PPM)
// ===========================================================================
bool decode_nexus(const std::vector<int> &durations, Rtl433Reading &out) {
    BitBuffer buf;
    if (!demod_ppm(durations, 500, 1000, 2000, 40, buf)) return false;
    if (buf.num_bits < 36) return false;

    // Byte 0: ID
    uint8_t id = buf.get_byte(0);
    if (id == 0x00 || id == 0xFF) return false;

    // Byte 1: Battery (bit 7), TX (bit 6), Channel (bits 5..4), Temp MSB (bits 3..0)
    uint8_t b1 = buf.get_byte(1);
    bool battery_low = (b1 & 0x80) != 0;
    uint8_t channel = ((b1 >> 4) & 0x03) + 1;

    // Byte 2: Temp LSB
    uint8_t b2 = buf.get_byte(2);
    int16_t temp_raw = ((b1 & 0x0F) << 8) | b2;
    if (temp_raw & 0x0800) temp_raw |= 0xF000; // Sign extension
    float temp_c = temp_raw / 10.0f;

    if (temp_c < -40.0f || temp_c > 70.0f) return false;

    // Byte 3 & 4: Humidity & check
    uint8_t b3 = buf.get_byte(3);
    uint8_t b4 = buf.get_byte(4);
    uint8_t humidity = ((b3 & 0x0F) << 4) | (b4 >> 4);
    if (humidity > 100) {
        humidity = b4;
    }
    if (humidity > 100) {
        humidity = b3 & 0x7F;
    }
    if (humidity > 100) return false;

    out.protocol = "FineOffset-WH2";
    out.model = "WH2 / Rubicson / TFA";
    out.decoder_name = "Nexus";
    out.decoder_id = 1;
    out.device_id = id;
    out.channel = channel;
    out.has_temp = true;
    out.temp_c = temp_c;
    out.temp_f = temp_c * 1.8f + 32.0f;
    out.has_humidity = (humidity > 0);
    out.humidity = (float)humidity;
    out.has_battery = true;
    out.battery_ok = !battery_low;
    out.bit_len = buf.num_bits;
    out.payload_hex = buf.to_hex();
    return true;
}

// ===========================================================================
// Decoder 2: Acurite 606TX (OOK PWM)
// ===========================================================================
bool decode_acurite_606tx(const std::vector<int> &durations, Rtl433Reading &out) {
    BitBuffer buf;
    if (!demod_pwm(durations, 200, 600, 400, 45, buf)) return false;
    if (buf.num_bits < 32) return false;

    uint8_t b0 = buf.get_byte(0);
    uint8_t b1 = buf.get_byte(1);
    uint8_t b2 = buf.get_byte(2);
    uint8_t b3 = buf.get_byte(3);

    // Checksum: sum of first 3 bytes modulo 256
    if (((b0 + b1 + b2) & 0xFF) != b3) return false;

    uint8_t id = b0;
    bool battery_low = (b1 & 0x80) != 0;
    uint8_t channel = ((b1 >> 4) & 0x07);

    int16_t temp_raw = ((b1 & 0x0F) << 8) | b2;
    float temp_c = (temp_raw - 1000) / 10.0f;
    if (temp_c < -40.0f || temp_c > 70.0f) {
        temp_c = (temp_raw - 1024) / 10.0f;
    }
    if (temp_c < -40.0f || temp_c > 70.0f) return false;

    out.protocol = "Acurite-606TX";
    out.model = "Acurite 606TX";
    out.decoder_name = "Acurite-606";
    out.decoder_id = 2;
    out.device_id = id;
    out.channel = channel + 1;
    out.has_temp = true;
    out.temp_c = temp_c;
    out.temp_f = temp_c * 1.8f + 32.0f;
    out.has_battery = true;
    out.battery_ok = !battery_low;
    out.bit_len = 32;
    out.payload_hex = buf.to_hex();
    return true;
}

// ===========================================================================
// Decoder 3: Acurite Tower / 592TXR / 06002M (OOK PWM)
// ===========================================================================
bool decode_acurite_tower(const std::vector<int> &durations, Rtl433Reading &out) {
    BitBuffer buf;
    if (!demod_pwm_space(durations, 400, 200, 400, 45, buf)) return false;
    if (buf.num_bits < 56) return false;

    uint8_t b0 = buf.get_byte(0);
    uint8_t b1 = buf.get_byte(1);
    uint8_t b2 = buf.get_byte(2);
    uint8_t b3 = buf.get_byte(3);
    uint8_t b4 = buf.get_byte(4);
    uint8_t b5 = buf.get_byte(5);
    uint8_t b6 = buf.get_byte(6);

    uint8_t sum = (b0 + b1 + b2 + b3 + b4 + b5) & 0xFF;
    uint8_t crc = buf.crc8(0x07, 0x00, 0, 48);
    if (sum != b6 && crc != b6) {
        // try 7-bit parity checksum
        uint8_t sum7 = (b0 + b1 + b2 + b3 + b4 + b5) & 0x7F;
        if (sum7 != (b6 & 0x7F)) return false;
    }

    uint16_t id = ((b0 & 0x3F) << 8) | b1;
    uint8_t ch_code = (b0 >> 6) & 0x03;
    uint8_t channel = (ch_code == 3) ? 1 : (ch_code == 2 ? 2 : 3);
    bool battery_low = (b2 & 0x40) == 0; // In 592TXR bit 6 is bat ok

    uint8_t humidity = b3 & 0x7F;
    int16_t temp_raw = ((b4 & 0x7F) << 7) | (b5 & 0x7F);
    float temp_c = (temp_raw - 1000) / 10.0f;
    if (temp_c < -40.0f || temp_c > 70.0f) {
        temp_c = ((temp_raw - 1000) * 0.1f - 32.0f) / 1.8f; // if sent in F
    }
    if (temp_c < -40.0f || temp_c > 70.0f) return false;
    if (humidity > 100) return false;

    out.protocol = "Acurite-Tower";
    out.model = "592TXR / Tower";
    out.decoder_name = "Acurite-Tower";
    out.decoder_id = 3;
    out.device_id = id;
    out.channel = channel;
    out.has_temp = true;
    out.temp_c = temp_c;
    out.temp_f = temp_c * 1.8f + 32.0f;
    out.has_humidity = (humidity > 0);
    out.humidity = (float)humidity;
    out.has_battery = true;
    out.battery_ok = !battery_low;
    out.bit_len = buf.num_bits;
    out.payload_hex = buf.to_hex();
    return true;
}

// ===========================================================================
// Decoder 4: Oregon Scientific v2.1 & v3 (OOK Manchester)
// ===========================================================================
bool decode_oregon_scientific(const std::vector<int> &durations, Rtl433Reading &out) {
    BitBuffer buf;
    if (!demod_manchester(durations, 488, 45, buf, false)) {
        if (!demod_manchester(durations, 488, 45, buf, true)) return false;
    }
    if (buf.num_bits < 60) return false;

    // Search for sync nibble 0xA (binary 1010)
    int sync_idx = buf.search_sync(0x0A, 4);
    if (sync_idx < 0 || (sync_idx + 56) > buf.num_bits) {
        sync_idx = buf.search_sync(0x05, 4);
        if (sync_idx < 0 || (sync_idx + 56) > buf.num_bits) return false;
    }

    uint16_t start = sync_idx + 4;
    // Extract sensor type ID (4 nibbles = 16 bits)
    uint16_t type = (uint16_t)buf.extract_bits(start, 16);
    if (type == 0x0000 || type == 0xFFFF) return false;

    uint8_t channel_raw = (uint8_t)buf.extract_bits(start + 16, 4);
    uint8_t channel = (channel_raw == 1) ? 1 : (channel_raw == 2 ? 2 : (channel_raw == 4 ? 3 : 1));
    uint8_t rolling_code = (uint8_t)buf.extract_bits(start + 20, 8);
    uint8_t flags = (uint8_t)buf.extract_bits(start + 28, 4);
    bool battery_low = (flags & 0x04) != 0;

    // Temperature BCD nibbles
    uint8_t t_tenths = (uint8_t)buf.extract_bits(start + 32, 4);
    uint8_t t_units = (uint8_t)buf.extract_bits(start + 36, 4);
    uint8_t t_tens = (uint8_t)buf.extract_bits(start + 40, 4);
    uint8_t t_sign = (uint8_t)buf.extract_bits(start + 44, 4);

    if (t_tenths > 9 || t_units > 9 || t_tens > 9) return false;
    float temp_c = (t_tens * 10 + t_units + t_tenths * 0.1f) * (t_sign ? -1.0f : 1.0f);
    if (temp_c < -50.0f || temp_c > 70.0f) return false;

    // Humidity BCD nibbles (if available)
    float humidity = 0.0f;
    bool has_hum = false;
    if (start + 56 <= buf.num_bits) {
        uint8_t h_units = (uint8_t)buf.extract_bits(start + 48, 4);
        uint8_t h_tens = (uint8_t)buf.extract_bits(start + 52, 4);
        if (h_units <= 9 && h_tens <= 9) {
            humidity = (float)(h_tens * 10 + h_units);
            if (humidity > 0 && humidity <= 100) has_hum = true;
        }
    }

    out.protocol = "Oregon-Scientific";
    char type_hex[8];
    snprintf(type_hex, sizeof(type_hex), "%04X", type);
    out.model = "Oregon " + String(type_hex);
    out.decoder_name = "Oregon-v2.1";
    out.decoder_id = 4;
    out.device_id = ((uint32_t)type << 8) | rolling_code;
    out.channel = channel;
    out.has_temp = true;
    out.temp_c = temp_c;
    out.temp_f = temp_c * 1.8f + 32.0f;
    out.has_humidity = has_hum;
    out.humidity = humidity;
    out.has_battery = true;
    out.battery_ok = !battery_low;
    out.bit_len = buf.num_bits;
    out.payload_hex = buf.to_hex();
    return true;
}

// ===========================================================================
// Decoder 5: Ambient Weather / Fine Offset WH65B / WH24 / WS-1000 (2-FSK PCM)
// ===========================================================================
bool decode_fineoffset_fsk(const std::vector<int> &durations, Rtl433Reading &out) {
    BitBuffer buf;
    // 17.24 kbps -> 58µs bit period
    if (!demod_pcm_fsk(durations, 58, 45, buf)) return false;
    if (buf.num_bits < 120) return false;

    // Search for sync word 0x2DD4 (16 bits)
    int sync_idx = buf.search_sync(0x2DD4, 16);
    if (sync_idx < 0) {
        sync_idx = buf.search_sync(0xD42D, 16);
        if (sync_idx < 0) return false;
    }

    uint16_t start = sync_idx + 16;
    if (start + 112 > buf.num_bits) return false;

    uint8_t b0 = (uint8_t)buf.extract_bits(start, 8);
    uint8_t b1 = (uint8_t)buf.extract_bits(start + 8, 8);
    uint8_t b2 = (uint8_t)buf.extract_bits(start + 16, 8);
    uint8_t b3 = (uint8_t)buf.extract_bits(start + 24, 8);
    uint8_t b4 = (uint8_t)buf.extract_bits(start + 32, 8);
    uint8_t b5 = (uint8_t)buf.extract_bits(start + 40, 8);
    uint8_t b6 = (uint8_t)buf.extract_bits(start + 48, 8);
    uint8_t b7 = (uint8_t)buf.extract_bits(start + 56, 8);
    uint8_t b8 = (uint8_t)buf.extract_bits(start + 64, 8);
    uint8_t b9 = (uint8_t)buf.extract_bits(start + 72, 8);
    uint8_t b10 = (uint8_t)buf.extract_bits(start + 80, 8);
    uint8_t b11 = (uint8_t)buf.extract_bits(start + 88, 8);
    uint8_t b12 = (uint8_t)buf.extract_bits(start + 96, 8);
    uint8_t crc_rx = (uint8_t)buf.extract_bits(start + 104, 8);

    uint8_t crc_calc = buf.crc8(0x31, 0x00, start, 104);
    if (crc_calc != crc_rx) {
        // Try relaxed check if packet matches known family
        if (b0 != 0x24 && b0 != 0x48 && b0 != 0x2B && b0 != 0x5B && b0 != 0x1B) {
            return false;
        }
    }

    uint16_t id = ((uint16_t)b1 << 8) | b2;
    int16_t temp_raw = ((b3 & 0x07) << 8) | b4;
    float temp_c = (temp_raw - 400) / 10.0f;
    uint8_t humidity = b5;

    if (temp_c < -45.0f || temp_c > 75.0f) return false;
    if (humidity > 100) return false;

    float wind_speed = ((float)b7) * 0.1f;
    float wind_gust = ((float)b8) * 0.1f;
    int16_t wind_dir = ((b3 & 0x80) ? 256 : 0) | b6;
    float rain = (float)(((uint16_t)b9 << 8) | b10) * 0.1f;
    float uv = (float)b11 * 0.1f;
    float solar = (float)b12 * 10.0f;

    out.protocol = "FineOffset-FSK";
    out.model = (b0 == 0x24) ? "WH24 Weather" : ((b0 == 0x48) ? "WH65B Station" : "FineOffset FSK");
    out.decoder_name = "FineOffset-FSK";
    out.decoder_id = 5;
    out.device_id = id;
    out.channel = 1;
    out.has_temp = true;
    out.temp_c = temp_c;
    out.temp_f = temp_c * 1.8f + 32.0f;
    out.has_humidity = (humidity > 0);
    out.humidity = (float)humidity;
    out.has_wind = true;
    out.wind_speed_ms = wind_speed;
    out.wind_gust_ms = wind_gust;
    out.wind_dir_deg = (wind_dir <= 360) ? wind_dir : -1;
    out.has_rain = (rain >= 0.0f && rain < 5000.0f);
    out.rain_mm = rain;
    out.has_uv = (uv >= 0.0f && uv <= 20.0f);
    out.uv_index = uv;
    out.solar_radiation = solar;
    out.has_battery = true;
    out.battery_ok = (b3 & 0x08) == 0;
    out.bit_len = buf.num_bits;
    out.payload_hex = buf.to_hex();
    return true;
}

// ===========================================================================
// Decoder 6: LaCrosse TX29 / TX35 (2-FSK / OOK)
// ===========================================================================
bool decode_lacrosse_tx(const std::vector<int> &durations, Rtl433Reading &out) {
    BitBuffer buf;
    if (!demod_pcm_fsk(durations, 104, 45, buf)) {
        if (!demod_ppm(durations, 500, 1000, 2000, 45, buf)) return false;
    }
    if (buf.num_bits < 40) return false;

    int sync_idx = buf.search_sync(0x0A, 4);
    if (sync_idx < 0 || sync_idx + 36 > buf.num_bits) return false;

    uint16_t start = sync_idx + 4;
    uint8_t id = (uint8_t)buf.extract_bits(start, 7);
    if (id == 0 || id == 0x7F) return false;

    uint8_t type = (uint8_t)buf.extract_bits(start + 7, 5);
    uint16_t temp_raw = (uint16_t)buf.extract_bits(start + 12, 12);
    float temp_c = (temp_raw - 500) / 10.0f;
    if (temp_c < -40.0f || temp_c > 70.0f) return false;

    out.protocol = "LaCrosse-TX";
    out.model = "LaCrosse TX29/TX35";
    out.decoder_name = "LaCrosse";
    out.decoder_id = 6;
    out.device_id = id;
    out.channel = 1;
    out.has_temp = true;
    out.temp_c = temp_c;
    out.temp_f = temp_c * 1.8f + 32.0f;
    out.has_battery = true;
    out.battery_ok = (type != 0);
    out.bit_len = buf.num_bits;
    out.payload_hex = buf.to_hex();
    return true;
}

// ===========================================================================
// Decoder 7: Honeywell / Ademco 5800 Security Sensors (OOK Manchester)
// ===========================================================================
bool decode_honeywell_5800(const std::vector<int> &durations, Rtl433Reading &out) {
    BitBuffer buf;
    if (!demod_manchester(durations, 380, 45, buf, false)) {
        if (!demod_manchester(durations, 380, 45, buf, true)) return false;
    }
    if (buf.num_bits < 56) return false;

    // Search for preamble / sync
    int sync_idx = buf.search_sync(0xFF, 8);
    uint16_t start = (sync_idx >= 0) ? (sync_idx + 8) : 0;
    if (start + 32 > buf.num_bits) return false;

    uint32_t serial = buf.extract_bits(start, 24);
    if (serial == 0 || serial == 0xFFFFFF) return false;

    uint8_t status = (uint8_t)buf.extract_bits(start + 24, 8);
    bool loop1 = (status & 0x80) != 0;
    bool loop2 = (status & 0x20) != 0;
    bool loop3 = (status & 0x40) != 0;
    bool tamper = (status & 0x10) != 0;
    bool bat_low = (status & 0x08) != 0;
    bool heartbeat = (status & 0x04) != 0;

    String state_str = "";
    if (tamper) state_str += "TAMPER ";
    if (loop1) state_str += "L1 ";
    if (loop2) state_str += "L2 ";
    if (loop3) state_str += "L3 ";
    if (bat_low) state_str += "LOWBAT ";
    if (heartbeat) state_str += "HEARTBEAT";
    if (state_str.length() == 0) state_str = "CLOSED / OK";

    out.protocol = "Honeywell-5800";
    out.model = "Ademco 5800 Sensor";
    out.decoder_name = "Honeywell";
    out.decoder_id = 7;
    out.device_id = serial;
    out.has_status = true;
    out.status_flags = status;
    out.status_str = state_str;
    out.has_battery = true;
    out.battery_ok = !bat_low;
    out.bit_len = buf.num_bits;
    out.payload_hex = buf.to_hex();
    return true;
}

// ===========================================================================
// Decoder 8: Schrader TPMS (OOK Manchester)
// ===========================================================================
bool decode_schrader_tpms(const std::vector<int> &durations, Rtl433Reading &out) {
    BitBuffer buf;
    if (!demod_manchester(durations, 120, 45, buf, false)) {
        if (!demod_manchester(durations, 120, 45, buf, true)) return false;
    }
    if (buf.num_bits < 56) return false;

    uint32_t id = buf.extract_bits(0, 32);
    if (id == 0 || id == 0xFFFFFFFF) return false;

    uint8_t pressure_raw = (uint8_t)buf.extract_bits(32, 8);
    uint8_t temp_raw = (uint8_t)buf.extract_bits(40, 8);
    uint8_t flags = (uint8_t)buf.extract_bits(48, 8);

    float psi = pressure_raw * 0.363f; // ~0.36 psi/count (typical Schrader)
    float kpa = psi * 6.89476f;
    float temp_c = (float)temp_raw - 50.0f;

    if (psi > 120.0f || temp_c < -40.0f || temp_c > 125.0f) return false;

    out.protocol = "Schrader-TPMS";
    out.model = "Schrader EG53MA4";
    out.decoder_name = "Schrader-TPMS";
    out.decoder_id = 8;
    out.device_id = id;
    out.has_pressure = true;
    out.pressure_kpa = kpa;
    out.pressure_psi = psi;
    out.has_temp = true;
    out.temp_c = temp_c;
    out.temp_f = temp_c * 1.8f + 32.0f;
    out.has_status = true;
    out.status_flags = flags;
    out.status_str = (flags & 0x10) ? "MOTION" : "STATIONARY";
    out.has_battery = true;
    out.battery_ok = (flags & 0x08) == 0;
    out.bit_len = buf.num_bits;
    out.payload_hex = buf.to_hex();
    return true;
}

// ===========================================================================
// Decoder 9: Toyota TPMS (2-FSK Manchester)
// ===========================================================================
bool decode_toyota_tpms(const std::vector<int> &durations, Rtl433Reading &out) {
    BitBuffer buf;
    if (!demod_manchester(durations, 104, 45, buf, false)) {
        if (!demod_manchester(durations, 104, 45, buf, true)) return false;
    }
    if (buf.num_bits < 64) return false;

    // Search sync 0x0155 or 0x0255
    int sync_idx = buf.search_sync(0x0155, 16);
    if (sync_idx < 0) sync_idx = buf.search_sync(0x0255, 16);
    if (sync_idx < 0) sync_idx = 0;

    uint16_t start = sync_idx;
    if (start + 56 > buf.num_bits) return false;

    uint32_t id = buf.extract_bits(start, 32);
    if (id == 0 || id == 0xFFFFFFFF) return false;

    uint8_t pressure_raw = (uint8_t)buf.extract_bits(start + 32, 8);
    uint8_t temp_raw = (uint8_t)buf.extract_bits(start + 40, 8);
    uint8_t status = (uint8_t)buf.extract_bits(start + 48, 8);

    float psi = (pressure_raw * 0.25f);
    float kpa = psi * 6.89476f;
    float temp_c = (float)temp_raw - 40.0f;

    if (psi > 100.0f || temp_c < -40.0f || temp_c > 125.0f) return false;

    out.protocol = "Toyota-TPMS";
    out.model = "Toyota / Lexus TPMS";
    out.decoder_name = "Toyota-TPMS";
    out.decoder_id = 9;
    out.device_id = id;
    out.has_pressure = true;
    out.pressure_kpa = kpa;
    out.pressure_psi = psi;
    out.has_temp = true;
    out.temp_c = temp_c;
    out.temp_f = temp_c * 1.8f + 32.0f;
    out.has_status = true;
    out.status_flags = status;
    out.status_str = "TIRE OK";
    out.has_battery = true;
    out.battery_ok = true;
    out.bit_len = buf.num_bits;
    out.payload_hex = buf.to_hex();
    return true;
}

// ===========================================================================
// Decoder 10: Kerui / EV1527 / PT2262 Security Sensors (OOK PWM)
// ===========================================================================
bool decode_kerui_ev1527(const std::vector<int> &durations, Rtl433Reading &out) {
    BitBuffer buf;
    if (!demod_pwm_space(durations, 350, 1050, 350, 45, buf)) {
        if (!demod_pwm(durations, 350, 1050, 350, 45, buf)) return false;
    }
    if (buf.num_bits < 24) return false;

    uint32_t raw24 = buf.extract_bits(0, 24);
    uint32_t addr = (raw24 >> 4) & 0xFFFFF;
    uint8_t cmd = raw24 & 0x0F;

    if (addr == 0 || addr == 0xFFFFF) return false;

    String state_str = "ALARM";
    if (cmd == 0x01) state_str = "TAMPER";
    else if (cmd == 0x02) state_str = "PANIC / SOS";
    else if (cmd == 0x04) state_str = "DOOR CLOSED";
    else if (cmd == 0x08) state_str = "DOOR OPEN / MOTION";
    else if (cmd == 0x09 || cmd == 0x0A) state_str = "LOW BATTERY";

    out.protocol = "EV1527-Security";
    out.model = "Kerui / EV1527 Sensor";
    out.decoder_name = "EV1527";
    out.decoder_id = 10;
    out.device_id = addr;
    out.has_status = true;
    out.status_flags = cmd;
    out.status_str = state_str;
    out.has_battery = true;
    out.battery_ok = (cmd != 0x09 && cmd != 0x0A);
    out.bit_len = 24;
    out.payload_hex = buf.to_hex();
    return true;
}

// ===========================================================================
// Decoder 11: DSC Security Sensors (OOK Manchester)
// ===========================================================================
bool decode_dsc_security(const std::vector<int> &durations, Rtl433Reading &out) {
    BitBuffer buf;
    if (!demod_manchester(durations, 500, 45, buf, false)) {
        if (!demod_manchester(durations, 500, 45, buf, true)) return false;
    }
    if (buf.num_bits < 32) return false;

    uint32_t esn = buf.extract_bits(0, 24);
    if (esn == 0 || esn == 0xFFFFFF) return false;

    uint8_t status = (uint8_t)buf.extract_bits(24, 4);
    uint8_t crc = (uint8_t)buf.extract_bits(28, 4);

    String state_str = "CLOSED";
    if (status & 0x01) state_str = "OPEN / ALARM";
    if (status & 0x04) state_str = "TAMPER";
    if (status & 0x08) state_str = "LOW BATTERY";

    out.protocol = "DSC-Security";
    out.model = "DSC WS4904 / Contact";
    out.decoder_name = "DSC";
    out.decoder_id = 11;
    out.device_id = esn;
    out.has_status = true;
    out.status_flags = (status << 4) | crc;
    out.status_str = state_str;
    out.has_battery = true;
    out.battery_ok = (status & 0x08) == 0;
    out.bit_len = 32;
    out.payload_hex = buf.to_hex();
    return true;
}

// ===========================================================================
// Decoder 12: Proove / Nexa / KlikAanKlikUit (OOK PWM)
// ===========================================================================
bool decode_proove_nexa(const std::vector<int> &durations, Rtl433Reading &out) {
    BitBuffer buf;
    if (!demod_pwm(durations, 250, 1250, 250, 45, buf)) return false;
    if (buf.num_bits < 32) return false;

    uint32_t raw = buf.extract_bits(0, 32);
    uint32_t tx_id = (raw >> 6) & 0x03FFFFFF;
    if (tx_id == 0 || tx_id == 0x03FFFFFF) return false;

    bool group = (raw >> 5) & 0x01;
    bool state = (raw >> 4) & 0x01;
    uint8_t unit = raw & 0x0F;

    out.protocol = "Proove-Nexa";
    out.model = "Nexa / Proove Remote";
    out.decoder_name = "Nexa";
    out.decoder_id = 12;
    out.device_id = tx_id;
    out.channel = unit;
    out.has_status = true;
    out.status_flags = (group ? 0x80 : 0) | (state ? 0x01 : 0);
    out.status_str = state ? "SWITCH ON" : "SWITCH OFF";
    if (group) out.status_str += " (ALL)";
    out.bit_len = 32;
    out.payload_hex = buf.to_hex();
    return true;
}

// ===========================================================================
// Decoder 13: Bresser 5-in-1 Weather Station (GFSK / 2-FSK PCM)
// ===========================================================================
// Bresser 5-in-1 outdoor sensor (temperature, humidity, wind speed, wind dir, rain, battery)
// Typically transmitted over 868.35 MHz / 433.92 MHz GFSK/FSK at ~8.21 kbps (122µs) or 17.24 kbps (58µs)
bool decode_bresser_5in1(const std::vector<int> &durations, Rtl433Reading &out) {
    BitBuffer buf;
    // Try 8.21 kbps (~122µs bit period) first, then 17.24 kbps (~58µs)
    if (!demod_pcm_fsk(durations, 122, 45, buf)) {
        if (!demod_pcm_fsk(durations, 58, 45, buf)) return false;
    }
    if (buf.num_bits < 80) return false;

    // Search for sync word: 0x2DD4 (16-bit) or 0xD4 (8-bit)
    int sync_idx = -1;
    for (int i = 0; i <= (int)buf.num_bits - 80; i++) {
        if (buf.extract_bits(i, 16) == 0x2DD4) {
            sync_idx = i + 16;
            break;
        } else if (buf.extract_bits(i, 8) == 0xD4) {
            sync_idx = i + 8;
            break;
        }
    }
    if (sync_idx < 0 || sync_idx + 80 > (int)buf.num_bits) return false;

    // Extract message bytes
    uint8_t bytes[12];
    for (int i = 0; i < 10; i++) {
        bytes[i] = (uint8_t)buf.extract_bits(sync_idx + i * 8, 8);
    }

    uint8_t sensor_id = bytes[0];
    if (sensor_id == 0x00 || sensor_id == 0xFF) return false;

    // Flags & Battery: Byte 1 (bit 7: battery ok flag, bit 6..4: flags)
    bool battery_ok = (bytes[1] & 0x80) != 0;

    // Temperature: Bytes 2..3 (BCD or binary offset)
    int16_t temp_raw = ((bytes[2] & 0x0F) << 8) | bytes[3];
    if (temp_raw & 0x0800) temp_raw |= 0xF000;
    float temp_c = temp_raw / 10.0f;
    if (temp_c < -40.0f || temp_c > 70.0f) {
        // BCD fallback
        int t_h = (bytes[2] >> 4) & 0x0F;
        int t_m = bytes[2] & 0x0F;
        int t_l = (bytes[3] >> 4) & 0x0F;
        temp_c = (t_h * 10.0f + t_m + t_l * 0.1f) - 40.0f;
    }
    if (temp_c < -40.0f || temp_c > 75.0f) return false;

    // Humidity: Byte 4
    uint8_t humidity = bytes[4];
    if (humidity > 100) return false;

    // Wind Direction & Speed: Bytes 5..6
    float wind_dir_deg = ((bytes[5] & 0x0F) * 22.5f);
    float wind_speed = ((bytes[6] & 0xFF) / 5.0f);

    // Rain accumulation counter: Bytes 7..8
    float rain_mm = (((bytes[7] << 8) | bytes[8]) * 0.4f);

    // Checksum: simple additive or CRC
    uint8_t check = bytes[9];
    uint8_t sum = 0;
    for (int i = 0; i < 9; i++) sum += bytes[i];
    if (sum != check && (sum & 0xFF) != check && (buf.crc8(0x31, 0x00, sync_idx, 9 * 8) != check)) {
        // Tolerant if sensor_id and valid temp/hum ranges match
        if (humidity == 0 || temp_c < -30.0f || temp_c > 65.0f) return false;
    }

    out.protocol = "Bresser-5in1";
    out.model = "Bresser 5-in-1 Station";
    out.decoder_name = "Bresser";
    out.decoder_id = 13;
    out.device_id = (bytes[0] << 8) | (bytes[1] & 0x0F);
    out.has_temp = true;
    out.temp_c = temp_c;
    out.temp_f = temp_c * 1.8f + 32.0f;
    out.has_humidity = (humidity > 0);
    out.humidity = (float)humidity;
    out.has_wind = true;
    out.wind_speed_ms = wind_speed;
    out.wind_gust_ms = wind_speed * 1.3f;
    out.wind_dir_deg = (int16_t)wind_dir_deg;
    out.has_rain = true;
    out.rain_mm = rain_mm;
    out.has_battery = true;
    out.battery_ok = battery_ok;
    out.bit_len = buf.num_bits;
    out.payload_hex = buf.to_hex();
    return true;
}

// ===========================================================================
// Decoder 14: Bresser 6-in-1 / 7-in-1 Weather Station (GFSK PCM)
// ===========================================================================
// Bresser 6-in-1 / 7-in-1 outdoor weather sensor (Temp, Hum, Wind, Gust, Rain, UV, Solar)
bool decode_bresser_6in1(const std::vector<int> &durations, Rtl433Reading &out) {
    BitBuffer buf;
    if (!demod_pcm_fsk(durations, 125, 45, buf)) { // 8.0 kbps
        if (!demod_pcm_fsk(durations, 58, 45, buf)) return false;
    }
    if (buf.num_bits < 144) return false;

    int sync_idx = -1;
    for (int i = 0; i <= (int)buf.num_bits - 144; i++) {
        if (buf.extract_bits(i, 16) == 0x2DD4 || buf.extract_bits(i, 16) == 0xAA2D) {
            sync_idx = i + 16;
            break;
        }
    }
    if (sync_idx < 0 || sync_idx + 128 > (int)buf.num_bits) return false;

    uint32_t station_id = (uint32_t)buf.extract_bits(sync_idx, 32);
    if (station_id == 0 || station_id == 0xFFFFFFFF) return false;

    uint8_t flags = (uint8_t)buf.extract_bits(sync_idx + 32, 8);
    bool battery_ok = (flags & 0x80) != 0;

    int16_t temp_raw = (int16_t)buf.extract_bits(sync_idx + 40, 16);
    float temp_c = (temp_raw - 400) / 10.0f;
    if (temp_c < -40.0f || temp_c > 75.0f) {
        temp_c = temp_raw / 10.0f;
    }
    if (temp_c < -40.0f || temp_c > 75.0f) return false;

    uint8_t humidity = (uint8_t)buf.extract_bits(sync_idx + 56, 8);
    if (humidity > 100) return false;

    float wind_speed = (float)buf.extract_bits(sync_idx + 64, 8) * 0.2f;
    float wind_gust = (float)buf.extract_bits(sync_idx + 72, 8) * 0.2f;
    int16_t wind_dir = (int16_t)(buf.extract_bits(sync_idx + 80, 8) * 1.40625f);
    float rain_mm = (float)buf.extract_bits(sync_idx + 88, 16) * 0.4f;
    float uv_idx = (float)buf.extract_bits(sync_idx + 104, 8) / 10.0f;
    float solar = (float)buf.extract_bits(sync_idx + 112, 16) * 0.1f;

    out.protocol = "Bresser-6in1";
    out.model = "Bresser 6-in-1 / 7-in-1";
    out.decoder_name = "Bresser";
    out.decoder_id = 14;
    out.device_id = station_id;
    out.has_temp = true;
    out.temp_c = temp_c;
    out.temp_f = temp_c * 1.8f + 32.0f;
    out.has_humidity = (humidity > 0);
    out.humidity = (float)humidity;
    out.has_wind = true;
    out.wind_speed_ms = wind_speed;
    out.wind_gust_ms = wind_gust;
    out.wind_dir_deg = wind_dir;
    out.has_rain = true;
    out.rain_mm = rain_mm;
    out.has_uv = true;
    out.uv_index = uv_idx;
    out.solar_radiation = solar;
    out.has_battery = true;
    out.battery_ok = battery_ok;
    out.bit_len = buf.num_bits;
    out.payload_hex = buf.to_hex();
    return true;
}

// ===========================================================================
// Decoder 15: Wireless M-Bus (wM-Bus EN 13757-4 Mode T / Mode S MSK)
// ===========================================================================
// Decodes smart utility meter telegrams (Water, Gas, Heat, Electricity meters)
// Mode T: 868.95 MHz MSK 100 kbps (10µs bit period)
// Mode S: 868.30 MHz MSK 32.768 kbps (30.5µs bit period Manchester)
bool decode_wmbus(const std::vector<int> &durations, Rtl433Reading &out) {
    BitBuffer buf;
    // Try Mode T (100 kbps -> 10µs bit period) first
    bool is_mode_t = demod_pcm_fsk(durations, 10, 45, buf);
    if (!is_mode_t || buf.num_bits < 80) {
        // Try Mode S (32.768 kbps -> 30.5µs bit period or 15.25µs Manchester half clock)
        if (!demod_manchester(durations, 15, 45, buf, false)) {
            if (!demod_pcm_fsk(durations, 30, 45, buf)) return false;
        }
    }
    if (buf.num_bits < 80) return false;

    // Search for wM-Bus Sync Word (0x543D or 0x2DD4 or 0x5555)
    int sync_idx = -1;
    for (int i = 0; i <= (int)buf.num_bits - 80; i++) {
        uint16_t w = (uint16_t)buf.extract_bits(i, 16);
        if (w == 0x543D || w == 0x2DD4 || w == 0xAA2D) {
            sync_idx = i + 16;
            break;
        }
    }
    if (sync_idx < 0) {
        // Try starting from offset 0 if length field looks valid
        uint8_t l0 = (uint8_t)buf.extract_bits(0, 8);
        if (l0 >= 10 && l0 <= 64 && (l0 * 8 <= (int)buf.num_bits)) {
            sync_idx = 0;
        } else {
            return false;
        }
    }
    if (sync_idx + 80 > (int)buf.num_bits) return false;

    uint8_t length = (uint8_t)buf.extract_bits(sync_idx, 8);
    if (length < 9 || length > 128) return false;

    uint8_t c_field = (uint8_t)buf.extract_bits(sync_idx + 8, 8);
    uint16_t manuf = (uint16_t)buf.extract_bits(sync_idx + 16, 16);
    // Swap endianness for manufacturer code
    manuf = ((manuf & 0xFF) << 8) | (manuf >> 8);

    char c1 = ((manuf >> 10) & 0x1F) + '@';
    char c2 = ((manuf >> 5) & 0x1F) + '@';
    char c3 = (manuf & 0x1F) + '@';
    String manuf_str = "";
    if (c1 >= 'A' && c1 <= 'Z') manuf_str += c1;
    if (c2 >= 'A' && c2 <= 'Z') manuf_str += c2;
    if (c3 >= 'A' && c3 <= 'Z') manuf_str += c3;
    if (manuf_str.length() < 2) manuf_str = "UNK";

    // 4-byte BCD Meter Serial Number (stored LSB first in standard wM-Bus)
    uint32_t raw_id = (uint32_t)buf.extract_bits(sync_idx + 32, 32);
    // Swap 32-bit endianness
    uint32_t bcd_id = ((raw_id & 0x000000FF) << 24) |
                      ((raw_id & 0x0000FF00) << 8)  |
                      ((raw_id & 0x00FF0000) >> 8)  |
                      ((raw_id & 0xFF000000) >> 24);

    uint8_t version = (uint8_t)buf.extract_bits(sync_idx + 64, 8);
    uint8_t dev_type = (uint8_t)buf.extract_bits(sync_idx + 72, 8);

    String type_str = "Meter";
    switch (dev_type) {
        case 0x01: type_str = "Oil Meter"; break;
        case 0x02: type_str = "Electricity Meter"; break;
        case 0x03: type_str = "Gas Meter"; break;
        case 0x04: type_str = "Heat Meter"; break;
        case 0x05: type_str = "Steam Meter"; break;
        case 0x06: type_str = "Warm Water Meter"; break;
        case 0x07: type_str = "Water Meter"; break;
        case 0x08: type_str = "Heat Cost Allocator"; break;
        case 0x09: type_str = "Compressed Air"; break;
        case 0x0E: type_str = "Cooling Meter"; break;
        case 0x15: type_str = "Smoke / Alarm"; break;
        case 0x16: type_str = "Room Sensor"; break;
        default:   type_str = "wM-Bus Meter"; break;
    }

    String c_name = "SND_NR";
    if (c_field == 0x44) c_name = "SND_NR";
    else if (c_field == 0x46) c_name = "SND_IR";
    else if (c_field == 0x47) c_name = "ACC_NR";
    else if (c_field == 0x7A) c_name = "REQ_UD2";
    else c_name = "0x" + String(c_field, HEX);

    out.protocol = "Wireless-MBus";
    out.model = "wM-Bus " + type_str;
    out.decoder_name = "wM-Bus";
    out.decoder_id = 15;
    out.device_id = bcd_id ? bcd_id : (uint32_t)raw_id;
    out.channel = dev_type;
    out.has_status = true;
    out.status_flags = (c_field << 8) | dev_type;
    out.status_str = "M:" + manuf_str + " [" + c_name + "] Ver:" + String(version);
    out.has_battery = true;
    out.battery_ok = true;
    out.bit_len = buf.num_bits;
    out.payload_hex = buf.to_hex();
    return true;
}
