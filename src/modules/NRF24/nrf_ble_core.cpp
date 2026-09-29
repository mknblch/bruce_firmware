#include "nrf_ble_core.h"
#include <esp_random.h>

static rf24_pa_dbm_e current_pa_level = RF24_PA_MAX;

uint8_t nrf_ble_chan_to_rf(uint8_t ble_chan) {
    switch (ble_chan) {
        case 37: return NRF_BLE_RF_CH37;
        case 38: return NRF_BLE_RF_CH38;
        case 39: return NRF_BLE_RF_CH39;
        default: return NRF_BLE_RF_CH37;
    }
}

uint8_t nrf_ble_rf_to_chan(uint8_t rf_chan) {
    switch (rf_chan) {
        case NRF_BLE_RF_CH37: return 37;
        case NRF_BLE_RF_CH38: return 38;
        case NRF_BLE_RF_CH39: return 39;
        default: return 37;
    }
}

uint8_t nrf_ble_swapbits(uint8_t a) {
    uint8_t v = 0;
    if (a & 0x80) v |= 0x01;
    if (a & 0x40) v |= 0x02;
    if (a & 0x20) v |= 0x04;
    if (a & 0x10) v |= 0x08;
    if (a & 0x08) v |= 0x10;
    if (a & 0x04) v |= 0x20;
    if (a & 0x02) v |= 0x40;
    if (a & 0x01) v |= 0x80;
    return v;
}

void nrf_ble_crc(const uint8_t *data, uint8_t len, uint8_t *dst) {
    uint8_t v, t, d;
    while (len--) {
        d = *data++;
        for (v = 0; v < 8; v++, d >>= 1) {
            t = dst[0] >> 7;
            dst[0] <<= 1;
            if (dst[1] & 0x80) dst[0] |= 1;
            dst[1] <<= 1;
            if (dst[2] & 0x80) dst[1] |= 1;
            dst[2] <<= 1;
            if (t != (d & 1)) {
                dst[2] ^= 0x5B;
                dst[1] ^= 0x06;
            }
        }
    }
}

void nrf_ble_whiten(uint8_t *data, uint8_t len, uint8_t whitenCoeff) {
    uint8_t m;
    while (len--) {
        for (m = 1; m; m <<= 1) {
            if (whitenCoeff & 0x80) {
                whitenCoeff ^= 0x11;
                (*data) ^= m;
            }
            whitenCoeff <<= 1;
        }
        data++;
    }
}

uint8_t nrf_ble_whiten_start(uint8_t chan) {
    return nrf_ble_swapbits(chan) | 2;
}

bool nrf_ble_init_radio() {
    NRF24_MODE mode = nrf_setMode();
    if (!nrf_start(mode)) {
        return false;
    }
    if (!CHECK_NRF_SPI(mode)) {
        return false;
    }

    NRFradio.setAutoAck(false);
    NRFradio.disableCRC();
    NRFradio.setAddressWidth(4);
    NRFradio.setDataRate(RF24_1MBPS);
    NRFradio.setPALevel(current_pa_level);
    NRFradio.setRetries(0, 0);
    NRFradio.setPayloadSize(32);
    NRFradio.flush_rx();
    NRFradio.flush_tx();

    // BLE Advertising Access Address = 0x8E89BED6
    // Bit-reversed & byte swapped for nRF24: 0x71, 0x91, 0x7D, 0x6B
    const uint8_t ble_addr[4] = {0x71, 0x91, 0x7D, 0x6B};
    NRFradio.openWritingPipe(ble_addr);
    NRFradio.openReadingPipe(0, ble_addr);
    return true;
}

void nrf_ble_deinit_radio() {
    NRFradio.stopListening();
    NRFradio.powerDown();
}

void nrf_ble_set_power(rf24_pa_dbm_e level) {
    current_pa_level = level;
    NRFradio.setPALevel(level);
}

void nrf_ble_random_mac(uint8_t *mac) {
    for (int i = 0; i < 6; i++) {
        mac[i] = (uint8_t)(esp_random() & 0xFF);
    }
    // Set random static address bits (MSB top 2 bits set to 1)
    mac[5] |= 0xC0;
}

String nrf_ble_mac_to_str(const uint8_t *mac) {
    char str[18];
    // MAC is passed in LSB-first (over the air) or MSB-first:
    // Format as AA:BB:CC:DD:EE:FF from mac[5] to mac[0]
    snprintf(str, sizeof(str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    return String(str);
}

bool nrf_ble_build_packet(uint8_t *out_buf, uint8_t &out_len, uint8_t pdu_type,
                          const uint8_t *mac, const uint8_t *adv_payload,
                          uint8_t adv_len, uint8_t ble_chan) {
    if (adv_len > 21) adv_len = 21; // Cap to fit in 32-byte nRF24 payload (2+6+21+3=32)

    uint8_t pdu_len = 8 + adv_len; // 2 header + 6 MAC + adv_len
    uint8_t total_len = pdu_len + 3; // + 3 CRC

    uint8_t raw[32];
    uint8_t header = pdu_type;
    // Set TxAdd = 1 (Random Address) if random MAC is used (MSB top 2 bits 11)
    if ((header & 0x40) == 0 && ((mac[5] & 0xC0) == 0xC0)) {
        header |= 0x40; // TxAdd = 1 (Random)
    }
    raw[0] = header;
    raw[1] = 6 + adv_len; // payload length field
    memcpy(&raw[2], mac, 6);
    if (adv_len > 0 && adv_payload != nullptr) {
        memcpy(&raw[8], adv_payload, adv_len);
    }

    // Pre-populate CRC with initial value 0x555555
    raw[pdu_len + 0] = 0x55;
    raw[pdu_len + 1] = 0x55;
    raw[pdu_len + 2] = 0x55;

    // Calculate CRC over pdu_len bytes into raw[pdu_len..pdu_len+2]
    nrf_ble_crc(raw, pdu_len, &raw[pdu_len]);

    // Swap CRC bits before whitening (per BLE spec / Dmitry Grinberg framing)
    for (uint8_t i = 0; i < 3; i++) {
        raw[pdu_len + i] = nrf_ble_swapbits(raw[pdu_len + i]);
    }

    // Whiten the entire packet (PDU + swapped CRC)
    nrf_ble_whiten(raw, total_len, nrf_ble_whiten_start(ble_chan));

    // Swap all bits for nRF24 transmission (nRF24 sends MSB first, BLE sends LSB first)
    for (uint8_t i = 0; i < total_len; i++) {
        out_buf[i] = nrf_ble_swapbits(raw[i]);
    }

    out_len = total_len;
    return true;
}

bool nrf_ble_send_raw(const uint8_t *packet, uint8_t len, uint8_t ble_chan) {
    uint8_t rf_ch = nrf_ble_chan_to_rf(ble_chan);
    NRFradio.setChannel(rf_ch);
    NRFradio.stopListening();
    NRFradio.flush_tx();
    return NRFradio.write(packet, len);
}

bool nrf_ble_send_adv(uint8_t pdu_type, const uint8_t *mac,
                      const uint8_t *adv_payload, uint8_t adv_len,
                      uint8_t ble_chan) {
    uint8_t tx_buf[32];
    uint8_t tx_len = 0;

    if (ble_chan == 0xFF) {
        // Broadcast across all 3 primary advertising channels: 37, 38, 39
        const uint8_t channels[3] = {37, 38, 39};
        bool success = true;
        for (int i = 0; i < 3; i++) {
            if (nrf_ble_build_packet(tx_buf, tx_len, pdu_type, mac, adv_payload, adv_len, channels[i])) {
                if (!nrf_ble_send_raw(tx_buf, tx_len, channels[i])) {
                    success = false;
                }
            }
        }
        return success;
    } else {
        if (nrf_ble_build_packet(tx_buf, tx_len, pdu_type, mac, adv_payload, adv_len, ble_chan)) {
            return nrf_ble_send_raw(tx_buf, tx_len, ble_chan);
        }
    }
    return false;
}

bool nrf_ble_send_adv_dual(uint8_t adv_pdu_type, const uint8_t *mac,
                           const uint8_t *adv_payload, uint8_t adv_len,
                           const uint8_t *rsp_payload, uint8_t rsp_len,
                           uint8_t ble_chan) {
    if (mac == nullptr || (adv_len > 0 && adv_payload == nullptr) ||
        (rsp_len > 0 && rsp_payload == nullptr) || adv_len > 21 || rsp_len > 21 ||
        (adv_pdu_type != NRF_BLE_ADV_IND && adv_pdu_type != NRF_BLE_ADV_SCAN_IND)) {
        return false;
    }

    auto send_channel_dual = [&](uint8_t ch) -> bool {
        uint8_t tx_buf[32];
        uint8_t tx_len = 0;
        if (!nrf_ble_build_packet(tx_buf, tx_len, adv_pdu_type, mac, adv_payload, adv_len, ch) ||
            !nrf_ble_send_raw(tx_buf, tx_len, ch)) {
            return false;
        }

        NRFradio.startListening();
        bool success = true;
        for (uint8_t attempt = 0; attempt < 100; attempt++) {
            if (NRFradio.available()) {
                uint8_t request_raw[32];
                NRFradio.read(request_raw, sizeof(request_raw));
                NrfBlePacket request;
                if (nrf_ble_parse_packet(request_raw, ch, request) && request.crc_ok &&
                    request.pdu_type == NRF_BLE_SCAN_REQ && request.adv_len >= 6 &&
                    memcmp(request.mac, mac, 6) == 0) {
                    NRFradio.stopListening();
                    delayMicroseconds(150); // T_IFS (Inter Frame Space)
                    if (!nrf_ble_build_packet(tx_buf, tx_len, NRF_BLE_SCAN_RSP, mac,
                                              rsp_payload, rsp_len, ch) ||
                        !nrf_ble_send_raw(tx_buf, tx_len, ch)) {
                        success = false;
                    }
                    break;
                }
            }
            delayMicroseconds(100);
        }
        NRFradio.stopListening();
        return success;
    };

    if (ble_chan == 0xFF) {
        const uint8_t channels[3] = {37, 38, 39};
        bool success = true;
        for (int i = 0; i < 3; i++) {
            if (!send_channel_dual(channels[i])) {
                success = false;
            }
        }
        return success;
    } else {
        if (ble_chan != 37 && ble_chan != 38 && ble_chan != 39) return false;
        return send_channel_dual(ble_chan);
    }
}

uint8_t nrf_ble_build_pcap_frame(const NrfBlePacket &pkt, uint8_t *out_buf, uint8_t out_capacity) {
    const uint8_t payload_len = pkt.raw[1] & 0x3F;
    if (out_buf == nullptr || payload_len < 6 || payload_len > 27) return 0;

    const uint8_t pdu_and_crc_len = 2 + payload_len + 3;
    const uint8_t frame_len = NRF_BLE_PCAP_PHDR_LEN + 4 + pdu_and_crc_len;
    if (out_capacity < frame_len) return 0;

    out_buf[0] = pkt.channel;
    out_buf[1] = 0x7F; // Signal power unavailable
    out_buf[2] = 0x7F; // Noise power unavailable
    out_buf[3] = 0;    // No access-address offenses
    out_buf[4] = 0xD6; // Reference access address, little-endian
    out_buf[5] = 0xBE;
    out_buf[6] = 0x89;
    out_buf[7] = 0x8E;

    uint16_t flags = 0x0001 | 0x0010 | 0x0020 | 0x0400; // Dewhitened, reference AA/offenses valid, CRC checked
    if (pkt.crc_ok) flags |= 0x0800;
    out_buf[8] = flags & 0xFF;
    out_buf[9] = flags >> 8;
    out_buf[10] = 0xD6; // Advertising access address, little-endian
    out_buf[11] = 0xBE;
    out_buf[12] = 0x89;
    out_buf[13] = 0x8E;
    memcpy(&out_buf[NRF_BLE_PCAP_PHDR_LEN + 4], pkt.raw, pdu_and_crc_len);
    return frame_len;
}

bool nrf_ble_parse_packet(const uint8_t *raw_32, uint8_t ble_chan, NrfBlePacket &pkt) {
    uint8_t unswapped[32];
    for (int i = 0; i < 32; i++) {
        unswapped[i] = nrf_ble_swapbits(raw_32[i]);
    }

    nrf_ble_whiten(unswapped, 32, nrf_ble_whiten_start(ble_chan));

    memcpy(pkt.raw, unswapped, 32);
    pkt.len = 32;
    pkt.channel = ble_chan;
    pkt.rf_channel = nrf_ble_chan_to_rf(ble_chan);
    pkt.timestamp = millis();

    pkt.pdu_type = unswapped[0] & 0x0F;
    pkt.tx_add = (unswapped[0] >> 6) & 0x01;
    pkt.rx_add = (unswapped[0] >> 7) & 0x01;

    uint8_t payload_len = unswapped[1] & 0x3F;

    if (payload_len < 6 || payload_len > 27) {
        return false;
    }

    memcpy(pkt.mac, &unswapped[2], 6);
    pkt.mac_str = nrf_ble_mac_to_str(pkt.mac);

    pkt.adv_len = payload_len - 6;
    if (pkt.adv_len > 21) pkt.adv_len = 21;
    if (pkt.adv_len > 0) {
        memcpy(pkt.adv_data, &unswapped[8], pkt.adv_len);
    }

    // Verify CRC
    uint8_t pdu_len = 2 + payload_len;
    if (pdu_len + 3 <= 32) {
        uint8_t calc_crc_bytes[3] = {0x55, 0x55, 0x55};
        nrf_ble_crc(unswapped, pdu_len, calc_crc_bytes);

        // In BLE encoding, CRC bytes are bit-reversed before whitening
        uint8_t rx_crc_bytes[3];
        rx_crc_bytes[0] = nrf_ble_swapbits(unswapped[pdu_len + 0]);
        rx_crc_bytes[1] = nrf_ble_swapbits(unswapped[pdu_len + 1]);
        rx_crc_bytes[2] = nrf_ble_swapbits(unswapped[pdu_len + 2]);

        pkt.calc_crc = ((uint32_t)calc_crc_bytes[0] << 16) | ((uint32_t)calc_crc_bytes[1] << 8) | calc_crc_bytes[2];
        pkt.packet_crc = ((uint32_t)rx_crc_bytes[0] << 16) | ((uint32_t)rx_crc_bytes[1] << 8) | rx_crc_bytes[2];
        pkt.crc_ok = (calc_crc_bytes[0] == rx_crc_bytes[0] &&
                      calc_crc_bytes[1] == rx_crc_bytes[1] &&
                      calc_crc_bytes[2] == rx_crc_bytes[2]);
    } else {
        pkt.calc_crc = 0;
        pkt.packet_crc = 0;
        pkt.crc_ok = false;
    }

    pkt.name = nrf_ble_parse_name(pkt.adv_data, pkt.adv_len);
    pkt.vendor = nrf_ble_parse_vendor(pkt.adv_data, pkt.adv_len, pkt.company_id);

    return true;
}

String nrf_ble_parse_name(const uint8_t *adv_data, uint8_t adv_len) {
    if (adv_len < 2 || adv_data == nullptr) return "";
    uint8_t idx = 0;
    while (idx < adv_len) {
        uint8_t len = adv_data[idx];
        if (len == 0 || idx + len >= adv_len + 1) break;
        uint8_t type = adv_data[idx + 1];
        if (type == 0x09 || type == 0x08) { // Complete or Shortened Local Name
            char name_buf[32];
            uint8_t name_len = len - 1;
            if (name_len > 31) name_len = 31;
            memcpy(name_buf, &adv_data[idx + 2], name_len);
            name_buf[name_len] = '\0';
            return String(name_buf);
        }
        idx += len + 1;
    }
    return "";
}

String nrf_ble_parse_vendor(const uint8_t *adv_data, uint8_t adv_len, uint16_t &company_id) {
    company_id = 0;
    if (adv_len < 3 || adv_data == nullptr) return "";
    uint8_t idx = 0;
    while (idx < adv_len) {
        uint8_t len = adv_data[idx];
        if (len == 0 || idx + len >= adv_len + 1) break;
        uint8_t type = adv_data[idx + 1];
        if (type == 0xFF && len >= 3) { // Manufacturer Specific Data
            company_id = adv_data[idx + 2] | ((uint16_t)adv_data[idx + 3] << 8);
            switch (company_id) {
                case 0x004C: return "Apple Inc.";
                case 0x0006: return "Microsoft";
                case 0x00E0: return "Google";
                case 0x0075: return "Samsung";
                case 0x038F: return "Xiaomi";
                case 0x02E5: return "Espressif";
                case 0x0001: return "Nokia";
                case 0x0002: return "Intel";
                case 0x000A: return "Qualcomm";
                case 0x0059: return "Nordic Semi";
                case 0x0087: return "Garmin";
                case 0x00D2: return "Dialog Semi";
                case 0x0157: return "Anhui Huami (Amazfit)";
                default: {
                    char buf[16];
                    snprintf(buf, sizeof(buf), "Mfg:0x%04X", company_id);
                    return String(buf);
                }
            }
        }
        if (type == 0x03 || type == 0x16) { // 16-bit Service UUID
            uint16_t uuid = adv_data[idx + 2] | ((uint16_t)adv_data[idx + 3] << 8);
            if (uuid == 0xFEAA) return "Eddystone";
            if (uuid == 0xFE2C) return "Google FastPair";
            if (uuid == 0xFD6F) return "Exposure Notification";
            if (uuid == 0xFEE0) return "Mi Band";
        }
        idx += len + 1;
    }
    return "";
}

String nrf_ble_pdu_type_str(uint8_t pdu_type) {
    switch (pdu_type) {
        case NRF_BLE_ADV_IND: return "ADV_IND";
        case NRF_BLE_ADV_DIRECT_IND: return "DIRECT_IND";
        case NRF_BLE_ADV_NONCONN_IND: return "NONCONN_IND";
        case NRF_BLE_SCAN_REQ: return "SCAN_REQ";
        case NRF_BLE_SCAN_RSP: return "SCAN_RSP";
        case NRF_BLE_CONNECT_IND: return "CONNECT_IND";
        case NRF_BLE_ADV_SCAN_IND: return "SCAN_IND";
        default: return "PDU_0x" + String(pdu_type, HEX);
    }
}

// ── Preset Packet Builders ───────────────────────────────────────────

uint8_t nrf_ble_build_ibeacon(uint8_t *buf, const uint8_t *uuid, uint16_t major, uint16_t minor, int8_t tx_power) {
    // Visual-only legacy preset: the 21-byte payload cap forces a truncated, non-compliant iBeacon block.
    uint8_t idx = 0;
    buf[idx++] = 0x14; // Length 20
    buf[idx++] = 0xFF; // Type: Manufacturer Specific
    buf[idx++] = 0x4C; // Apple CID LSB
    buf[idx++] = 0x00; // Apple CID MSB
    buf[idx++] = 0x02; // iBeacon Type
    buf[idx++] = 0x0F; // Subtype Length (15 bytes: 10 byte UUID + 2 Major + 2 Minor + 1 Pwr)
    memcpy(&buf[idx], uuid, 10);
    idx += 10;
    buf[idx++] = (major >> 8) & 0xFF;
    buf[idx++] = major & 0xFF;
    buf[idx++] = (minor >> 8) & 0xFF;
    buf[idx++] = minor & 0xFF;
    buf[idx++] = (uint8_t)tx_power;
    return idx;
}

uint8_t nrf_ble_build_eddystone_url(uint8_t *buf, const String &url, int8_t tx_power) {
    uint8_t idx = 0;
    // Flags: Len 2, Type 0x01, Val 0x06
    buf[idx++] = 0x02;
    buf[idx++] = 0x01;
    buf[idx++] = 0x06;

    // 16-bit Service UUID: Len 3, Type 0x03, 0xAA, 0xFE (Eddystone 0xFEAA)
    buf[idx++] = 0x03;
    buf[idx++] = 0x03;
    buf[idx++] = 0xAA;
    buf[idx++] = 0xFE;

    // URL Scheme prefixes (Eddystone URL Spec):
    // 0x00: "http://www."
    // 0x01: "https://www."
    // 0x02: "http://"
    // 0x03: "https://"
    uint8_t scheme = 0x03; // default "https://"
    String cleanUrl = url;
    if (cleanUrl.startsWith("http://www.")) { scheme = 0x00; cleanUrl = cleanUrl.substring(11); }
    else if (cleanUrl.startsWith("https://www.")) { scheme = 0x01; cleanUrl = cleanUrl.substring(12); }
    else if (cleanUrl.startsWith("http://")) { scheme = 0x02; cleanUrl = cleanUrl.substring(7); }
    else if (cleanUrl.startsWith("https://")) { scheme = 0x03; cleanUrl = cleanUrl.substring(8); }

    // Eddystone URL Suffix Token Table (ordered by longest match first)
    static const struct {
        const char *suffix;
        uint8_t code;
    } SUFFIX_TOKENS[] = {
        {".com/", 0x00},
        {".org/", 0x01},
        {".edu/", 0x02},
        {".net/", 0x03},
        {".info/", 0x04},
        {".biz/", 0x05},
        {".gov/", 0x06},
        {".com",  0x07},
        {".org",  0x08},
        {".edu",  0x09},
        {".net",  0x0A},
        {".info", 0x0B},
        {".biz",  0x0C},
        {".gov",  0x0D}
    };

    uint8_t encoded_url[21];
    uint8_t enc_len = 0;
    // Total adv payload limit is 21 bytes.
    // Fixed bytes: 3 (Flags) + 4 (UUID) + 1 (Service Data len) + 6 (Service Data Header) = 14 bytes
    // Max encoded url bytes = 21 - 14 = 7 bytes.
    const uint8_t max_enc_len = 7;

    size_t i = 0;
    while (i < cleanUrl.length() && enc_len < max_enc_len) {
        bool matched = false;
        for (const auto &token : SUFFIX_TOKENS) {
            size_t tlen = strlen(token.suffix);
            if (cleanUrl.substring(i).startsWith(token.suffix)) {
                encoded_url[enc_len++] = token.code;
                i += tlen;
                matched = true;
                break;
            }
        }
        if (!matched) {
            encoded_url[enc_len++] = (uint8_t)cleanUrl[i++];
        }
    }

    uint8_t service_len = 6 + enc_len; // 1 (0x16) + 2 (UUID) + 1 (Frame) + 1 (tx_power) + 1 (scheme) + enc_len
    buf[idx++] = service_len;
    buf[idx++] = 0x16; // Service Data - 16-bit UUID
    buf[idx++] = 0xAA; // UUID LSB
    buf[idx++] = 0xFE; // UUID MSB
    buf[idx++] = 0x10; // Frame: URL
    buf[idx++] = (uint8_t)tx_power;
    buf[idx++] = scheme;
    for (uint8_t j = 0; j < enc_len; j++) {
        buf[idx++] = encoded_url[j];
    }
    return idx;
}

uint8_t nrf_ble_build_eddystone_uid(uint8_t *buf, const uint8_t *nid, const uint8_t *bid, int8_t tx_power) {
    uint8_t idx = 0;
    // 16-bit Service UUID (4 bytes)
    buf[idx++] = 0x03;
    buf[idx++] = 0x03;
    buf[idx++] = 0xAA;
    buf[idx++] = 0xFE;

    // Service Data (UID Frame)
    // Emitted bytes: 0x16, 0xAA, 0xFE, 0x00, tx_power, 6 bytes nid, 4 bytes bid -> total 15 bytes
    buf[idx++] = 0x0F; // Length 15 (1 type + 2 uuid + 1 frame + 1 pwr + 6 nid + 4 bid)
    buf[idx++] = 0x16; // Service Data
    buf[idx++] = 0xAA;
    buf[idx++] = 0xFE;
    buf[idx++] = 0x00; // UID Frame Type
    buf[idx++] = (uint8_t)tx_power;
    // Visual-only compact preset: shortened identifiers do not meet the Eddystone UID specification.
    memcpy(&buf[idx], nid, 6); // Namespace (truncated to 6 for 21B payload fit)
    idx += 6;
    memcpy(&buf[idx], bid, 4); // Instance (4 bytes)
    idx += 4;
    return idx;
}

uint8_t nrf_ble_build_altbeacon(uint8_t *buf, const uint8_t *beacon_id, uint16_t mfg_id, int8_t ref_rssi) {
    uint8_t idx = 0;
    // Visual-only legacy preset: shortened ID and field ordering are not AltBeacon-compliant.
    buf[idx++] = 0x13; // Length 19 (1 type + 2 mfg + 2 code + 12 beacon_id + 1 rssi + 1 rsv)
    buf[idx++] = 0xFF; // Manufacturer Specific
    buf[idx++] = mfg_id & 0xFF;
    buf[idx++] = (mfg_id >> 8) & 0xFF;
    buf[idx++] = 0xBE; // AltBeacon Code
    buf[idx++] = 0xAC;
    memcpy(&buf[idx], beacon_id, 12); // Beacon ID 12 bytes
    idx += 12;
    buf[idx++] = (uint8_t)ref_rssi;
    buf[idx++] = 0x00; // Reserved
    return idx;
}

uint8_t nrf_ble_build_bruce_beacon(uint8_t *buf, const String &name) {
    uint8_t idx = 0;
    // Flags
    buf[idx++] = 0x02;
    buf[idx++] = 0x01;
    buf[idx++] = 0x06;

    // Complete Local Name
    String n = name;
    if (n.length() > 10) n = n.substring(0, 10);
    buf[idx++] = n.length() + 1;
    buf[idx++] = 0x09; // Complete Local Name
    for (size_t i = 0; i < n.length(); i++) {
        buf[idx++] = n[i];
    }

    // Custom Bruce Service
    if (idx + 4 <= 21) {
        buf[idx++] = 0x03;
        buf[idx++] = 0x03;
        buf[idx++] = 0xBC; // Bruce Custom UUID 0xBCBC
        buf[idx++] = 0xBC;
    }
    return idx;
}

void nrf_ble_build_bruce_beacon_dual(uint8_t *adv_buf, uint8_t &adv_len,
                                     uint8_t *rsp_buf, uint8_t &rsp_len,
                                     const String &name) {
    adv_len = 0;
    rsp_len = 0;
    if (adv_buf == nullptr || rsp_buf == nullptr) return;

    adv_buf[adv_len++] = 0x02;
    adv_buf[adv_len++] = 0x01;
    adv_buf[adv_len++] = 0x06;

    String n = name;
    if (n.length() > 10) n = n.substring(0, 10);
    adv_buf[adv_len++] = n.length() + 1;
    adv_buf[adv_len++] = 0x09;
    for (size_t i = 0; i < n.length(); i++) {
        adv_buf[adv_len++] = n[i];
    }

    rsp_buf[rsp_len++] = 0x03;
    rsp_buf[rsp_len++] = 0x03;
    rsp_buf[rsp_len++] = 0xBC;
    rsp_buf[rsp_len++] = 0xBC;
}

// ── Notification Builders ───────────────────────────────────────────

uint8_t nrf_ble_build_apple_notification(uint8_t *buf, uint8_t notification_type) {
    uint8_t idx = 0;
    buf[idx++] = 0x02;
    buf[idx++] = 0x01;
    buf[idx++] = 0x1A; // Flags: General Discoverable + BR/EDR not supported

    switch (notification_type) {
        case 0: { // AirPods Pro Setup
            buf[idx++] = 0x10; // Length 16
            buf[idx++] = 0xFF; // Mfg Data
            buf[idx++] = 0x4C; // Apple
            buf[idx++] = 0x00;
            buf[idx++] = 0x07; // Proximity Pair
            buf[idx++] = 0x0F; // Length
            buf[idx++] = 0x01; // Model AirPods Pro
            buf[idx++] = 0x0E;
            buf[idx++] = 0x20;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            break;
        }
        case 1: { // AirTag Proximity
            buf[idx++] = 0x10;
            buf[idx++] = 0xFF;
            buf[idx++] = 0x4C;
            buf[idx++] = 0x00;
            buf[idx++] = 0x12; // FindMy / AirTag
            buf[idx++] = 0x02;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            break;
        }
        case 2: { // AppleTV Setup
            buf[idx++] = 0x08;
            buf[idx++] = 0xFF;
            buf[idx++] = 0x4C;
            buf[idx++] = 0x00;
            buf[idx++] = 0x04; // Setup
            buf[idx++] = 0x04;
            buf[idx++] = 0x2A; // Apple TV
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            break;
        }
        case 3: { // iPhone Setup / Transfer Number
            buf[idx++] = 0x08;
            buf[idx++] = 0xFF;
            buf[idx++] = 0x4C;
            buf[idx++] = 0x00;
            buf[idx++] = 0x02; // Proximity
            buf[idx++] = 0x04;
            buf[idx++] = 0x02; // Phone number transfer
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            break;
        }
        default: { // Apple Watch
            buf[idx++] = 0x08;
            buf[idx++] = 0xFF;
            buf[idx++] = 0x4C;
            buf[idx++] = 0x00;
            buf[idx++] = 0x04;
            buf[idx++] = 0x04;
            buf[idx++] = 0x01; // Watch setup
            buf[idx++] = 0x00;
            buf[idx++] = 0x00;
            break;
        }
    }
    return idx;
}

uint8_t nrf_ble_build_google_fastpair(uint8_t *buf, uint32_t model_id) {
    uint8_t idx = 0;
    // Flags
    buf[idx++] = 0x02;
    buf[idx++] = 0x01;
    buf[idx++] = 0x06;

    // Complete 16-bit Service UUID (0xFE2C)
    buf[idx++] = 0x03;
    buf[idx++] = 0x03;
    buf[idx++] = 0x2C;
    buf[idx++] = 0xFE;

    // Service Data (Model ID)
    buf[idx++] = 0x06; // Length 6 (1 type + 2 uuid + 3 model)
    buf[idx++] = 0x16; // Service Data
    buf[idx++] = 0x2C;
    buf[idx++] = 0xFE;
    buf[idx++] = (model_id >> 16) & 0xFF;
    buf[idx++] = (model_id >> 8) & 0xFF;
    buf[idx++] = model_id & 0xFF;

    return idx;
}

uint8_t nrf_ble_build_samsung_setup(uint8_t *buf, uint8_t setup_type) {
    uint8_t idx = 0;
    // Flags
    buf[idx++] = 0x02;
    buf[idx++] = 0x01;
    buf[idx++] = 0x06;

    // Samsung Manufacturer Data
    buf[idx++] = 0x0D; // Length 13
    buf[idx++] = 0xFF; // Mfg Data
    buf[idx++] = 0x75; // Samsung CID 0x0075
    buf[idx++] = 0x00;
    buf[idx++] = 0x01;
    buf[idx++] = 0x00;
    buf[idx++] = 0x02;
    buf[idx++] = (setup_type == 0) ? 0x00 : 0x01; // Galaxy Buds or Watch
    buf[idx++] = 0x01;
    buf[idx++] = 0x01;
    buf[idx++] = 0xFF;
    buf[idx++] = 0x00;
    buf[idx++] = 0x00;
    buf[idx++] = 0x43;

    return idx;
}

uint8_t nrf_ble_build_swift_pair(uint8_t *buf, const String &device_name) {
    uint8_t idx = 0;
    // Flags
    buf[idx++] = 0x02;
    buf[idx++] = 0x01;
    buf[idx++] = 0x06;

    // Microsoft Swift Pair Mfg Data
    buf[idx++] = 0x06; // Length 6
    buf[idx++] = 0xFF;
    buf[idx++] = 0x06; // Microsoft CID 0x0006
    buf[idx++] = 0x00;
    buf[idx++] = 0x03; // Swift Pair Sub-Scenario
    buf[idx++] = 0x01; // RSSI Threshold
    buf[idx++] = 0x80; // Flags

    // Device Name
    String n = device_name;
    if (n.length() > 6) n = n.substring(0, 6);
    buf[idx++] = n.length() + 1;
    buf[idx++] = 0x09; // Complete Name
    for (size_t i = 0; i < n.length(); i++) {
        buf[idx++] = n[i];
    }

    return idx;
}
