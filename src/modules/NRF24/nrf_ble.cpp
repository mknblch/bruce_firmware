#include "nrf_ble.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/sd_functions.h"
#include <SD.h>
#include <vector>
#include <globals.h>

// ── Shared Settings ─────────────────────────────────────────────────
static rf24_pa_dbm_e ble_pa_level = RF24_PA_MAX;
static const char *pa_level_names[] = {"MIN", "LOW", "HIGH", "MAX"};
static uint8_t ble_active_channel_mode = 0; // 0: Hop (37/38/39), 1: Ch37, 2: Ch38, 3: Ch39

// ══════════════════════════════════════════════════════════════════
// ═══════════════════════ PCAP FILE WRITER ══════════════════════════
// ══════════════════════════════════════════════════════════════════

class NrfBlePcapWriter {
public:
    File file;
    bool active = false;
    String filename;

    bool begin() {
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
            String path = "/BrucePCAP/nrf_ble_" + String(idx) + ".pcap";
            if (!fs->exists(path.c_str())) {
                filename = path;
                break;
            }
            idx++;
        }

        file = fs->open(filename.c_str(), FILE_WRITE);
        if (!file) {
            active = false;
            return false;
        }

        // Global PCAP Header (LINKTYPE_BLUETOOTH_LE_LL = 251)
        uint32_t magic_number = 0xa1b2c3d4;
        uint16_t version_major = 2;
        uint16_t version_minor = 4;
        uint32_t thiszone = 0;
        uint32_t sigfigs = 0;
        uint32_t snaplen = 65535;
        uint32_t network = 251; // LINKTYPE_BLUETOOTH_LE_LL

        file.write((const uint8_t *)&magic_number, 4);
        file.write((const uint8_t *)&version_major, 2);
        file.write((const uint8_t *)&version_minor, 2);
        file.write((const uint8_t *)&thiszone, 4);
        file.write((const uint8_t *)&sigfigs, 4);
        file.write((const uint8_t *)&snaplen, 4);
        file.write((const uint8_t *)&network, 4);
        file.flush();

        active = true;
        return true;
    }

    void writePacket(const NrfBlePacket &pkt) {
        if (!active || !file) return;

        // In LINKTYPE_BLUETOOTH_LE_LL (251):
        // Layout: Access Address (4B) + PDU Header (1B) + PDU Len (1B) + AdvA MAC (6B) + AdvData (NB) + CRC (3B)
        uint8_t wire_buf[48];

        // Access Address for BLE Advertising (0x8E89BED6)
        wire_buf[0] = 0xD6;
        wire_buf[1] = 0xBE;
        wire_buf[2] = 0x89;
        wire_buf[3] = 0x8E;

        // PDU Header
        wire_buf[4] = pkt.raw[0];
        // PDU Length (6 + adv_len)
        wire_buf[5] = pkt.raw[1];
        // AdvA
        memcpy(&wire_buf[6], pkt.mac, 6);
        // Payload
        if (pkt.adv_len > 0) {
            memcpy(&wire_buf[12], pkt.adv_data, pkt.adv_len);
        }

        uint8_t pdu_len = 2 + 6 + pkt.adv_len;

        // CRC (3 bytes)
        wire_buf[4 + pdu_len + 0] = (pkt.packet_crc >> 16) & 0xFF;
        wire_buf[4 + pdu_len + 1] = (pkt.packet_crc >> 8) & 0xFF;
        wire_buf[4 + pdu_len + 2] = pkt.packet_crc & 0xFF;

        uint32_t total_wire_len = 4 + pdu_len + 3;

        // PCAP Record Header
        uint32_t ts_sec = pkt.timestamp / 1000;
        uint32_t ts_usec = (pkt.timestamp % 1000) * 1000;
        uint32_t incl_len = total_wire_len;
        uint32_t orig_len = total_wire_len;

        file.write((const uint8_t *)&ts_sec, 4);
        file.write((const uint8_t *)&ts_usec, 4);
        file.write((const uint8_t *)&incl_len, 4);
        file.write((const uint8_t *)&orig_len, 4);
        file.write(wire_buf, total_wire_len);
        file.flush();
    }

    void end() {
        if (active && file) {
            file.flush();
            file.close();
        }
        active = false;
    }
};

// ══════════════════════════════════════════════════════════════════
// ═══════════════════ SCROLLABLE DETAILS VIEWER ════════════════════
// ══════════════════════════════════════════════════════════════════

static void add_hex_ascii_dump(std::vector<String> &lines, const uint8_t *data, uint8_t len) {
    for (uint8_t i = 0; i < len; i += 8) {
        uint8_t chunk_len = (len - i >= 8) ? 8 : (len - i);
        char hex_buf[36];
        char asc_buf[24];
        char hex_part[26] = "";
        char asc_part[12] = "";

        for (uint8_t j = 0; j < chunk_len; j++) {
            char hb[4];
            snprintf(hb, sizeof(hb), "%02X ", data[i + j]);
            strcat(hex_part, hb);
            uint8_t c = data[i + j];
            asc_part[j] = (c >= 32 && c <= 126) ? (char)c : '.';
            asc_part[j + 1] = '\0';
        }
        snprintf(hex_buf, sizeof(hex_buf), "%02X: %s", i, hex_part);
        snprintf(asc_buf, sizeof(asc_buf), "  ASC: %s", asc_part);
        lines.push_back(String(hex_buf));
        lines.push_back(String(asc_buf));
    }
}

static void nrf_ble_show_scrollable_details(const String &header_title, const std::vector<String> &lines) {
    drawMainBorder(true);
    int scroll_offset = 0;
    int lineH = FP * LH + 1;
    int maxLines = (tftHeight - BORDER_PAD_Y - 20) / lineH;
    if (maxLines < 1) maxLines = 1;
    bool needs_redraw = true;

    while (true) {
        if (check(EscPress) || check(SelPress)) break;

        if (check(PrevPress) || check(UpPress)) {
            if (scroll_offset > 0) {
                scroll_offset--;
                needs_redraw = true;
            }
        }
        if (check(NextPress) || check(DownPress)) {
            if (scroll_offset + maxLines < (int)lines.size()) {
                scroll_offset++;
                needs_redraw = true;
            }
        }

        if (needs_redraw) {
            needs_redraw = false;

            tft.setTextSize(FP);
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.fillRect(BORDER_PAD_X, BORDER_PAD_Y, tftWidth - 2 * BORDER_PAD_X, lineH, bruceConfig.bgColor);
            String title_text = header_title + " [" + String(scroll_offset + 1) + "/" + String(lines.size()) + "]";
            tft.drawString(title_text, BORDER_PAD_X + 2, BORDER_PAD_Y);

            int startY = BORDER_PAD_Y + lineH + 1;
            for (int i = 0; i < maxLines; i++) {
                int lineIdx = scroll_offset + i;
                int currentY = startY + i * lineH;
                tft.fillRect(BORDER_PAD_X, currentY, tftWidth - 2 * BORDER_PAD_X, lineH, bruceConfig.bgColor);

                if (lineIdx < (int)lines.size()) {
                    const String &l = lines[lineIdx];
                    if (l.startsWith("---") || l.startsWith("[")) {
                        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
                    } else if (l.startsWith("MAC:") || l.startsWith("Name:") || l.startsWith("Vendor:") ||
                               l.startsWith("Total") || l.startsWith("Last") || l.startsWith("Channel:") ||
                               l.startsWith("PDU") || l.startsWith("CRC:") || l.startsWith("Adv")) {
                        tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
                    } else if (l.startsWith("  ASC:")) {
                        tft.setTextColor(TFT_GREEN, bruceConfig.bgColor);
                    } else {
                        tft.setTextColor(TFT_LIGHTGREY, bruceConfig.bgColor);
                    }
                    tft.drawString(l, BORDER_PAD_X + 2, currentY);
                }
            }

            printCenterFootnote("[UP/DN] Scroll  [ESC] Back");
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ══════════════════════════════════════════════════════════════════
// ═══════════════════════ 1. BLE SCANNER ════════════════════════════
// ══════════════════════════════════════════════════════════════════

struct ScannedBleDevice {
    NrfBlePacket packet;
    std::vector<NrfBlePacket> packets; // Keep last 8 packets per device
    uint16_t packet_count;
    unsigned long last_seen;
};

static void nrf_ble_inspect_device(const ScannedBleDevice &dev) {
    std::vector<String> lines;
    lines.push_back("MAC: " + dev.packet.mac_str + (dev.packet.tx_add ? " (Rand)" : " (Pub)"));
    lines.push_back("Name: " + (dev.packet.name.length() > 0 ? dev.packet.name : "<None>"));
    lines.push_back("Vendor: " + (dev.packet.vendor.length() > 0 ? dev.packet.vendor : "Unknown"));
    lines.push_back("Total Pkts: " + String(dev.packet_count) + " (Saved: " + String(dev.packets.size()) + ")");
    lines.push_back("Last Seen: " + String((millis() - dev.last_seen) / 1000) + "s ago");

    for (size_t p = 0; p < dev.packets.size(); p++) {
        // Show newest packets first
        const NrfBlePacket &pkt = dev.packets[dev.packets.size() - 1 - p];
        lines.push_back("--- Pkt #" + String(p + 1) + " (Ch " + String(pkt.channel) + ") ---");
        lines.push_back("Type: " + nrf_ble_pdu_type_str(pkt.pdu_type) + " (" + String(pkt.adv_len) + "B)");

        if (pkt.adv_len > 0) {
            add_hex_ascii_dump(lines, pkt.adv_data, pkt.adv_len);
        } else {
            lines.push_back("  <No Adv Payload>");
        }
    }

    nrf_ble_show_scrollable_details("Device Details", lines);
}

void nrf_ble_scanner() {
    if (!nrf_ble_init_radio()) {
        displayError("NRF24 not available", true);
        return;
    }

    std::vector<ScannedBleDevice> devices;
    const uint8_t channels[] = {37, 38, 39};
    uint8_t ch_idx = 0;
    int selected_idx = 0;
    int scroll_offset = 0;
    unsigned long last_ui_update = 0;
    bool needs_redraw = true;

    // Use clean border without overlapping title
    drawMainBorder(true);

    while (true) {
        if (check(EscPress)) break;

        if (check(PrevPress) || check(UpPress)) {
            if (selected_idx > 0) {
                selected_idx--;
                needs_redraw = true;
            }
        }
        if (check(NextPress) || check(DownPress)) {
            if (selected_idx + 1 < (int)devices.size()) {
                selected_idx++;
                needs_redraw = true;
            }
        }
        if (check(SelPress)) {
            if (devices.size() > 0 && selected_idx >= 0 && selected_idx < (int)devices.size()) {
                nrf_ble_inspect_device(devices[selected_idx]);
                drawMainBorder(true);
                needs_redraw = true;
            }
        }

        // Cycle through BLE advertising channels
        uint8_t current_ble_ch = channels[ch_idx];
        uint8_t rf_ch = nrf_ble_chan_to_rf(current_ble_ch);
        ch_idx = (ch_idx + 1) % 3;

        NRFradio.setChannel(rf_ch);
        NRFradio.startListening();

        for (int tries = 0; tries < 6; tries++) {
            delayMicroseconds(500);
            if (NRFradio.available()) {
                uint8_t raw[32];
                NRFradio.read(raw, 32);
                NrfBlePacket pkt;
                // Only show data with passed CRC
                if (nrf_ble_parse_packet(raw, current_ble_ch, pkt) && pkt.crc_ok) {
                    bool found = false;
                    for (size_t i = 0; i < devices.size(); i++) {
                        if (memcmp(devices[i].packet.mac, pkt.mac, 6) == 0) {
                            devices[i].packet_count++;
                            devices[i].last_seen = millis();
                            devices[i].packet.channel = pkt.channel;
                            if (pkt.name.length() > 0 && devices[i].packet.name.length() == 0) {
                                devices[i].packet.name = pkt.name;
                            }
                            if (pkt.vendor.length() > 0 && devices[i].packet.vendor.length() == 0) {
                                devices[i].packet.vendor = pkt.vendor;
                            }
                            // Store up to 8 recent packets per device
                            if (devices[i].packets.size() >= 8) {
                                devices[i].packets.erase(devices[i].packets.begin());
                            }
                            devices[i].packets.push_back(pkt);
                            devices[i].packet = pkt;
                            found = true;
                            break;
                        }
                    }
                    if (!found && devices.size() < 60) {
                        ScannedBleDevice dev;
                        dev.packet = pkt;
                        dev.packets.push_back(pkt);
                        dev.packet_count = 1;
                        dev.last_seen = millis();
                        devices.push_back(dev);
                        needs_redraw = true;
                    }
                }
            }
        }
        NRFradio.stopListening();

        // UI rendering
        if (millis() - last_ui_update > 250 || needs_redraw) {
            last_ui_update = millis();
            needs_redraw = false;

            int lineH = FP * LH + 1;
            int maxVisibleLines = (tftHeight - BORDER_PAD_Y - 22) / lineH;
            if (maxVisibleLines < 1) maxVisibleLines = 1;

            if (selected_idx < scroll_offset) scroll_offset = selected_idx;
            if (selected_idx >= scroll_offset + maxVisibleLines) {
                scroll_offset = selected_idx - maxVisibleLines + 1;
            }

            // Draw status banner inside top area without shadowing
            tft.setTextSize(FP);
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.fillRect(7, BORDER_PAD_Y, tftWidth - 14, lineH, bruceConfig.bgColor);
            String headerInfo = "Scan Ch:" + String(current_ble_ch) + "  Devs:" + String(devices.size()) + " (CRC OK)";
            tft.drawString(headerInfo, 10, BORDER_PAD_Y);

            int y = BORDER_PAD_Y + lineH;

            for (int i = 0; i < maxVisibleLines; i++) {
                int devIdx = scroll_offset + i;
                int currentY = y + i * lineH;

                tft.fillRect(7, currentY, tftWidth - 14, lineH, bruceConfig.bgColor);

                if (devIdx < (int)devices.size()) {
                    const ScannedBleDevice &dev = devices[devIdx];
                    bool isSelected = (devIdx == selected_idx);

                    if (isSelected) {
                        tft.fillRect(7, currentY, tftWidth - 14, lineH, bruceConfig.priColor);
                        tft.setTextColor(bruceConfig.bgColor, bruceConfig.priColor);
                    } else {
                        tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
                    }

                    String label = dev.packet.mac_str.substring(9); // Last 3 bytes of MAC
                    if (dev.packet.name.length() > 0) {
                        label += " " + dev.packet.name;
                    } else if (dev.packet.vendor.length() > 0) {
                        label += " " + dev.packet.vendor;
                    } else {
                        label += " " + nrf_ble_pdu_type_str(dev.packet.pdu_type);
                    }
                    if (label.length() > 21) label = label.substring(0, 21);

                    label += " #" + String(dev.packet_count);
                    tft.drawString(label, 10, currentY);
                }
            }

            printCenterFootnote("[ESC] Stop  [OK] Details");
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    nrf_ble_deinit_radio();
}

// ══════════════════════════════════════════════════════════════════
// ═══════════════════════ 2. BLE BEACON ═════════════════════════════
// ══════════════════════════════════════════════════════════════════

static void nrf_ble_run_beacon(const String &beaconName, uint8_t pdu_type,
                               const uint8_t *mac, const uint8_t *payload,
                               uint8_t payload_len) {
    if (!nrf_ble_init_radio()) {
        displayError("NRF24 not available", true);
        return;
    }

    drawMainBorder(true);

    unsigned long pkts_sent = 0;
    unsigned long start_time = millis();
    unsigned long last_stats = millis();
    float pps = 0.0;
    bool is_paused = false;
    uint8_t anim_frame = 0;

    int y = BORDER_PAD_Y + 4;
    int lineH = FP * LH + 2;

    while (true) {
        if (check(EscPress)) break;
        if (check(SelPress)) {
            is_paused = !is_paused;
            delay(150);
        }

        if (!is_paused) {
            uint8_t target_chan = 0xFF; // Hop across all 3
            if (ble_active_channel_mode == 1) target_chan = 37;
            else if (ble_active_channel_mode == 2) target_chan = 38;
            else if (ble_active_channel_mode == 3) target_chan = 39;

            nrf_ble_send_adv(pdu_type, mac, payload, payload_len, target_chan);
            pkts_sent += (target_chan == 0xFF ? 3 : 1);
        }

        if (millis() - last_stats > 200) {
            float elapsed = (millis() - start_time) / 1000.0f;
            if (elapsed > 0.1f) pps = pkts_sent / elapsed;
            last_stats = millis();
            anim_frame = (anim_frame + 1) % 4;

            tft.setTextSize(FP);

            // Row 1: Type
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString("Beacon: ", 10, y);
            tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
            tft.drawString(beaconName, 60, y);

            // Row 2: Status & Animation
            int r2 = y + lineH;
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString("Status: ", 10, r2);
            if (is_paused) {
                tft.setTextColor(TFT_YELLOW, bruceConfig.bgColor);
                tft.drawString("PAUSED       ", 60, r2);
            } else {
                tft.setTextColor(TFT_GREEN, bruceConfig.bgColor);
                String anim = "TX ACTIVE ";
                for (int a = 0; a <= anim_frame; a++) anim += ">";
                while (anim.length() < 16) anim += " ";
                tft.drawString(anim, 60, r2);
            }

            // Row 3: MAC Address
            int r3 = y + lineH * 2;
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString("MAC: ", 10, r3);
            tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
            tft.drawString(nrf_ble_mac_to_str(mac), 45, r3);

            // Row 4: Stats
            int r4 = y + lineH * 3;
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString("Pkts: ", 10, r4);
            tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
            String statsStr = String(pkts_sent) + " (" + String((int)pps) + " p/s)";
            tft.drawString(statsStr + "    ", 45, r4);

            // Row 5: RF config
            int r5 = y + lineH * 4;
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString("RF: ", 10, r5);
            tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
            String rfInfo = "PA:" + String(pa_level_names[ble_pa_level]) + " Ch:" +
                            (ble_active_channel_mode == 0 ? "Hop 37-39" : String(36 + ble_active_channel_mode));
            tft.drawString(rfInfo + "   ", 35, r5);

            printCenterFootnote(is_paused ? "[OK] Resume  [ESC] Stop" : "[OK] Pause   [ESC] Stop");
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }

    nrf_ble_deinit_radio();
}

void nrf_ble_beacon_menu() {
    uint8_t mac[6];
    nrf_ble_random_mac(mac);

    options = {
        {"iBeacon (AirTag)",       [&]() {
            uint8_t payload[24];
            const uint8_t airtag_uuid[10] = {0x4C, 0x00, 0x12, 0x19, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
            uint8_t len = nrf_ble_build_ibeacon(payload, airtag_uuid, 1, 1, -59);
            nrf_ble_run_beacon("iBeacon (AirTag)", NRF_BLE_ADV_NONCONN_IND, mac, payload, len);
        }},
        {"iBeacon (Proximity)",    [&]() {
            uint8_t payload[24];
            const uint8_t prox_uuid[10] = {0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2, 0xB0, 0x60};
            uint8_t len = nrf_ble_build_ibeacon(payload, prox_uuid, 100, 1, -59);
            nrf_ble_run_beacon("iBeacon (Prox)", NRF_BLE_ADV_NONCONN_IND, mac, payload, len);
        }},
        {"Eddystone URL (bruce.dev)", [&]() {
            uint8_t payload[24];
            uint8_t len = nrf_ble_build_eddystone_url(payload, "https://bruce.dev", -18);
            nrf_ble_run_beacon("Eddystone URL", NRF_BLE_ADV_NONCONN_IND, mac, payload, len);
        }},
        {"Eddystone URL (Custom)", [&]() {
            String url = keyboard("https://", 30, "Enter URL:");
            if (url.length() > 0 && url != "\x1B") {
                uint8_t payload[24];
                uint8_t len = nrf_ble_build_eddystone_url(payload, url, -18);
                nrf_ble_run_beacon("Eddystone Custom", NRF_BLE_ADV_NONCONN_IND, mac, payload, len);
            }
        }},
        {"Eddystone UID",          [&]() {
            uint8_t payload[24];
            const uint8_t nid[6] = {0xED, 0xDD, 0x11, 0x22, 0x33, 0x44};
            const uint8_t bid[4] = {0x00, 0x00, 0x00, 0x01};
            uint8_t len = nrf_ble_build_eddystone_uid(payload, nid, bid, -18);
            nrf_ble_run_beacon("Eddystone UID", NRF_BLE_ADV_NONCONN_IND, mac, payload, len);
        }},
        {"AltBeacon",              [&]() {
            uint8_t payload[24];
            const uint8_t bid[12] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C};
            uint8_t len = nrf_ble_build_altbeacon(payload, bid, 0x0118, -59);
            nrf_ble_run_beacon("AltBeacon", NRF_BLE_ADV_NONCONN_IND, mac, payload, len);
        }},
        {"Bruce Beacon",           [&]() {
            uint8_t payload[24];
            uint8_t len = nrf_ble_build_bruce_beacon(payload, "Bruce-NRF");
            nrf_ble_run_beacon("Bruce Beacon", NRF_BLE_ADV_NONCONN_IND, mac, payload, len);
        }},
    };

    loopOptions(options, MENU_TYPE_SUBMENU, "BLE Beacon");
}

// ══════════════════════════════════════════════════════════════════
// ═══════════════════════ 3. BLE SNIFFER ════════════════════════════
// ══════════════════════════════════════════════════════════════════

static void nrf_ble_inspect_packet(const NrfBlePacket &pkt) {
    std::vector<String> lines;
    lines.push_back("MAC: " + pkt.mac_str + (pkt.tx_add ? " (Rand)" : " (Pub)"));
    lines.push_back("Name: " + (pkt.name.length() > 0 ? pkt.name : "<None>"));
    lines.push_back("Vendor: " + (pkt.vendor.length() > 0 ? pkt.vendor : "Unknown"));
    lines.push_back("Channel: Ch" + String(pkt.channel) + " (RF " + String(pkt.rf_channel) + ")");
    lines.push_back("PDU Type: " + nrf_ble_pdu_type_str(pkt.pdu_type));
    lines.push_back("CRC: " + String(pkt.crc_ok ? "PASSED (OK)" : "FAILED") + " [0x" + String(pkt.packet_crc, HEX) + "]");
    lines.push_back("Adv Length: " + String(pkt.adv_len) + " Bytes");
    lines.push_back("--- Payload Dump ---");
    if (pkt.adv_len > 0) {
        add_hex_ascii_dump(lines, pkt.adv_data, pkt.adv_len);
    } else {
        lines.push_back("  <No Adv Payload>");
    }

    lines.push_back("--- Raw PDU Bytes ---");
    add_hex_ascii_dump(lines, pkt.raw, 32);

    nrf_ble_show_scrollable_details("Packet Details", lines);
}

void nrf_ble_sniffer() {
    if (!nrf_ble_init_radio()) {
        displayError("NRF24 not available", true);
        return;
    }

    // Ring buffer of last 50 packets
    std::vector<NrfBlePacket> packet_log;
    const uint8_t channels[] = {37, 38, 39};
    uint8_t ch_idx = 0;
    unsigned long total_pkts = 0;
    unsigned long valid_crc_pkts = 0;
    unsigned long last_ui_update = 0;
    bool is_paused = false;
    int selected_pkt_idx = 0;
    int scroll_offset = 0;
    bool needs_redraw = true;

    // Direct PCAP logging
    NrfBlePcapWriter pcap;
    bool pcap_ok = pcap.begin();

    drawMainBorder(true);

    while (true) {
        if (check(EscPress)) {
            if (is_paused) {
                is_paused = false;
                drawMainBorder(true);
                needs_redraw = true;
                delay(150);
            } else {
                break;
            }
        }

        if (check(SelPress)) {
            if (!is_paused) {
                is_paused = true;
                selected_pkt_idx = packet_log.size() > 0 ? (int)packet_log.size() - 1 : 0;
                scroll_offset = 0;
                drawMainBorder(true);
                needs_redraw = true;
            } else {
                // In paused mode, SelPress opens full scrollable packet details
                if (packet_log.size() > 0 && selected_pkt_idx >= 0 && selected_pkt_idx < (int)packet_log.size()) {
                    nrf_ble_inspect_packet(packet_log[selected_pkt_idx]);
                    drawMainBorder(true);
                    needs_redraw = true;
                }
            }
            delay(150);
        }

        if (is_paused) {
            if (check(PrevPress) || check(UpPress)) {
                if (selected_pkt_idx > 0) {
                    selected_pkt_idx--;
                    needs_redraw = true;
                }
            }
            if (check(NextPress) || check(DownPress)) {
                if (selected_pkt_idx + 1 < (int)packet_log.size()) {
                    selected_pkt_idx++;
                    needs_redraw = true;
                }
            }
        } else {
            // Live sniffing on advertising channels
            uint8_t current_ble_ch = channels[ch_idx];
            uint8_t rf_ch = nrf_ble_chan_to_rf(current_ble_ch);
            ch_idx = (ch_idx + 1) % 3;

            NRFradio.setChannel(rf_ch);
            NRFradio.startListening();

            for (int tries = 0; tries < 8; tries++) {
                delayMicroseconds(400);
                if (NRFradio.available()) {
                    uint8_t raw[32];
                    NRFradio.read(raw, 32);
                    NrfBlePacket pkt;
                    if (nrf_ble_parse_packet(raw, current_ble_ch, pkt)) {
                        total_pkts++;
                        if (pkt.crc_ok) valid_crc_pkts++;

                        // Ring buffer of last 50 packets
                        if (packet_log.size() >= 50) {
                            packet_log.erase(packet_log.begin());
                        }
                        packet_log.push_back(pkt);

                        // Direct PCAP file logging
                        if (pcap_ok) {
                            pcap.writePacket(pkt);
                        }
                    }
                }
            }
            NRFradio.stopListening();
        }

        // Render UI
        if (millis() - last_ui_update > 200 || needs_redraw) {
            last_ui_update = millis();
            needs_redraw = false;

            int lineH = FP * LH + 1;
            int maxLines = (tftHeight - BORDER_PAD_Y - 22) / lineH;
            if (maxLines < 1) maxLines = 1;
            int startY = BORDER_PAD_Y + lineH;

            tft.setTextSize(FP);
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.fillRect(7, BORDER_PAD_Y, tftWidth - 14, lineH, bruceConfig.bgColor);

            String topStatus = "Sniff:" + String(total_pkts) + " CRC:" + String(valid_crc_pkts);
            if (pcap_ok) topStatus += " [PCAP]";
            if (is_paused) topStatus += " [PAUSE]";
            tft.drawString(topStatus, 10, BORDER_PAD_Y);

            if (is_paused && packet_log.size() > 0) {
                // Browse ring buffer list of 50 packets
                if (selected_pkt_idx < scroll_offset) scroll_offset = selected_pkt_idx;
                if (selected_pkt_idx >= scroll_offset + maxLines) {
                    scroll_offset = selected_pkt_idx - maxLines + 1;
                }

                for (int i = 0; i < maxLines; i++) {
                    int pktIdx = scroll_offset + i;
                    int currentY = startY + i * lineH;
                    tft.fillRect(7, currentY, tftWidth - 14, lineH, bruceConfig.bgColor);

                    if (pktIdx < (int)packet_log.size()) {
                        const NrfBlePacket &pkt = packet_log[pktIdx];
                        bool isSelected = (pktIdx == selected_pkt_idx);

                        if (isSelected) {
                            tft.fillRect(7, currentY, tftWidth - 14, lineH, bruceConfig.priColor);
                            tft.setTextColor(bruceConfig.bgColor, bruceConfig.priColor);
                        } else {
                            tft.setTextColor(pkt.crc_ok ? TFT_WHITE : TFT_DARKGREY, bruceConfig.bgColor);
                        }

                        String line = String(pktIdx + 1) + ". 3" + String(pkt.channel % 10) + " " + pkt.mac_str.substring(9) + " ";
                        if (pkt.name.length() > 0) line += pkt.name;
                        else if (pkt.vendor.length() > 0) line += pkt.vendor;
                        else line += nrf_ble_pdu_type_str(pkt.pdu_type);

                        if (line.length() > 22) line = line.substring(0, 22);
                        line += (pkt.crc_ok ? " OK" : " ERR");
                        tft.drawString(line, 10, currentY);
                    }
                }

                printCenterFootnote("[UP/DN] Select  [OK] Dump  [ESC] Resume");
            } else {
                // Live streaming log (showing most recent entries)
                int displayCount = packet_log.size() < (size_t)maxLines ? packet_log.size() : maxLines;
                int startIdx = packet_log.size() - displayCount;

                for (int i = 0; i < maxLines; i++) {
                    int currentY = startY + i * lineH;
                    tft.fillRect(7, currentY, tftWidth - 14, lineH, bruceConfig.bgColor);

                    if (i < displayCount) {
                        const NrfBlePacket &pkt = packet_log[startIdx + i];
                        tft.setTextColor(pkt.crc_ok ? TFT_WHITE : TFT_DARKGREY, bruceConfig.bgColor);

                        String line = "3" + String(pkt.channel % 10) + " " + pkt.mac_str.substring(9) + " ";
                        if (pkt.name.length() > 0) line += pkt.name;
                        else if (pkt.vendor.length() > 0) line += pkt.vendor;
                        else line += nrf_ble_pdu_type_str(pkt.pdu_type);

                        if (line.length() > 22) line = line.substring(0, 22);
                        line += (pkt.crc_ok ? " OK" : " ERR");
                        tft.drawString(line, 10, currentY);
                    }
                }
                printCenterFootnote("[OK] Pause/Ring [ESC] Stop");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    pcap.end();
    nrf_ble_deinit_radio();
}

// ══════════════════════════════════════════════════════════════════
// ═══════════════���═══════ 4. NOTIFICATIONS ══════════════════════════
// ══════════════════════════════════════════════════════════════════

static void nrf_ble_run_notifications(const String &suiteName, uint8_t notificationMode) {
    if (!nrf_ble_init_radio()) {
        displayError("NRF24 not available", true);
        return;
    }

    drawMainBorder(true);

    unsigned long pkts_sent = 0;
    unsigned long start_time = millis();
    unsigned long last_stats = millis();
    float pps = 0.0;
    bool is_paused = false;
    uint8_t subtype = 0;
    uint8_t anim_frame = 0;

    int y = BORDER_PAD_Y + 4;
    int lineH = FP * LH + 2;

    uint8_t mac[6];
    nrf_ble_random_mac(mac);

    while (true) {
        if (check(EscPress)) break;
        if (check(SelPress)) {
            is_paused = !is_paused;
            delay(150);
        }

        if (!is_paused) {
            uint8_t payload[24];
            uint8_t len = 0;

            // Rotate MAC periodically for notification realism
            if (pkts_sent % 30 == 0) {
                nrf_ble_random_mac(mac);
            }

            switch (notificationMode) {
                case 0: // Apple Continuity
                    len = nrf_ble_build_apple_notification(payload, subtype % 5);
                    subtype++;
                    break;
                case 1: // Google Fast Pair
                    len = nrf_ble_build_google_fastpair(payload, 0xF582A5 + (subtype % 4));
                    subtype++;
                    break;
                case 2: // Samsung Easy Setup
                    len = nrf_ble_build_samsung_setup(payload, subtype % 2);
                    subtype++;
                    break;
                case 3: // Microsoft Swift Pair
                    len = nrf_ble_build_swift_pair(payload, "Surface");
                    break;
                default: // Cycle All
                    if (subtype % 4 == 0) len = nrf_ble_build_apple_notification(payload, 0);
                    else if (subtype % 4 == 1) len = nrf_ble_build_google_fastpair(payload, 0xF582A5);
                    else if (subtype % 4 == 2) len = nrf_ble_build_samsung_setup(payload, 0);
                    else len = nrf_ble_build_swift_pair(payload, "Mouse");
                    subtype++;
                    break;
            }

            if (len > 0) {
                nrf_ble_send_adv(NRF_BLE_ADV_NONCONN_IND, mac, payload, len, 0xFF);
                pkts_sent += 3;
            }
        }

        if (millis() - last_stats > 200) {
            float elapsed = (millis() - start_time) / 1000.0f;
            if (elapsed > 0.1f) pps = pkts_sent / elapsed;
            last_stats = millis();
            anim_frame = (anim_frame + 1) % 4;

            tft.setTextSize(FP);

            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString("Alert: ", 10, y);
            tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
            tft.drawString(suiteName, 55, y);

            int r2 = y + lineH;
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString("Status: ", 10, r2);
            if (is_paused) {
                tft.setTextColor(TFT_YELLOW, bruceConfig.bgColor);
                tft.drawString("PAUSED       ", 60, r2);
            } else {
                tft.setTextColor(TFT_GREEN, bruceConfig.bgColor);
                String anim = "BROADCAST ";
                for (int a = 0; a <= anim_frame; a++) anim += ">";
                while (anim.length() < 16) anim += " ";
                tft.drawString(anim, 60, r2);
            }

            int r3 = y + lineH * 2;
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString("MAC: ", 10, r3);
            tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
            tft.drawString(nrf_ble_mac_to_str(mac), 45, r3);

            int r4 = y + lineH * 3;
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString("Pkts: ", 10, r4);
            tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
            String statsStr = String(pkts_sent) + " (" + String((int)pps) + " p/s)";
            tft.drawString(statsStr + "    ", 45, r4);

            int r5 = y + lineH * 4;
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString("Channels: ", 10, r5);
            tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
            tft.drawString("37, 38, 39 (Hop)", 70, r5);

            printCenterFootnote(is_paused ? "[OK] Resume  [ESC] Stop" : "[OK] Pause   [ESC] Stop");
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    nrf_ble_deinit_radio();
}

void nrf_ble_notification_menu() {
    options = {
        {"Apple Continuity",    [&]() { nrf_ble_run_notifications("Apple Popups", 0); }},
        {"Google Fast Pair",   [&]() { nrf_ble_run_notifications("Google FastPair", 1); }},
        {"Samsung Easy Setup",  [&]() { nrf_ble_run_notifications("Samsung Setup", 2); }},
        {"Microsoft Swift Pair",[&]() { nrf_ble_run_notifications("MS SwiftPair", 3); }},
        {"Multi-Vendor Spam",   [&]() { nrf_ble_run_notifications("Multi-Vendor", 4); }},
    };

    loopOptions(options, MENU_TYPE_SUBMENU, "BLE Notification");
}

// ══════════════════════════════════════════════════════════════════
// ═══════════════════════ MAIN NRF24BLE MENU ════════════════════════
// ══════════════════════════════════════════════════════════════════

void nrf_ble_menu() {
    options = {
        {"Scanner",      nrf_ble_scanner},
        {"Beacon",       nrf_ble_beacon_menu},
        {"Sniffing",     nrf_ble_sniffer},
        {"Notification", nrf_ble_notification_menu},
        {"PA Power: " + String(pa_level_names[ble_pa_level]), [&]() {
            ble_pa_level = (rf24_pa_dbm_e)((ble_pa_level + 1) % 4);
            nrf_ble_set_power(ble_pa_level);
            nrf_ble_menu();
        }},
    };

    loopOptions(options, MENU_TYPE_SUBMENU, "NRF24BLE");
}
