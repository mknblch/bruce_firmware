#if !defined(LITE_VERSION)
#include "LoRaScanner.h"
#include "LoRaConfig.h"
#include "LoRaRadio.h"
#include "LoRaSniffer.h"
#include "LoRaTracker.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/utils.h"
#include <Arduino.h>

struct ScanChannelEntry {
    String label;
    float freqMHz;
    uint8_t sf;
    float bwKHz;
    uint32_t hits;
    float peakRssi;
    float lastRssi;
    uint32_t lastSeenMs;
    uint8_t syncWord = 0;
    bool hasSyncWord = false;
};

static constexpr uint32_t LORA_SCAN_DWELL_MS = 1500;

static String formatSyncWord(uint8_t syncWord) {
    char buffer[3];
    snprintf(buffer, sizeof(buffer), "%02X", syncWord);
    return String(buffer);
}

static void drainKeyboardInput() {
    vTaskDelay(pdMS_TO_TICKS(150));
    SelPress = false;
    EscPress = false;
    PrevPress = false;
    NextPress = false;
    UpPress = false;
    DownPress = false;
    AnyKeyPress = false;
    NextPagePress = false;
    PrevPagePress = false;
    KeyStroke.Clear();
}

void runLoRaChannelDetector() {
    loadLoRaConfig();
    if (!isLoraHardwareConfigured()) {
        displayError("LoRa pins not configured!", true);
        return;
    }

    displayTextLine("Init Channel Detector...");

    std::vector<ScanChannelEntry> channels;
    uint8_t scanCr = loraConfig.cr;
    uint8_t scanSyncWord = loraConfig.syncWord;
    uint16_t scanPreambleLen = loraConfig.preambleLen;

    // Pick scan mode
    std::vector<Option> scanModes = {
        {"Waveshare HF-B 868.0 (SF7/BW125)", [&]() {
            channels = {
                {"Waveshare 868.0", 868.000f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
            };
            scanCr = 5;
            scanSyncWord = 0x12;
            scanPreambleLen = 8;
        }},
        {"Bruce 868.1 Test (SF9/BW31)", [&]() {
            channels = {
                {"Bruce 868.1", 868.100f, 9, 31.25f, 0, -140.0f, -140.0f, 0},
            };
            scanCr = 8;
            scanSyncWord = 0x12;
            scanPreambleLen = 8;
        }},
        {"Meshtastic Presets", [&]() {
            scanCr = 5;
            scanSyncWord = 0x2B;
            scanPreambleLen = 16;
            channels = {
                {"EU868 LongFast", 869.525f, 11, 250.0f, 0, -140.0f, -140.0f, 0, 0x2B, true},
                {"EU868 MedFast",  869.525f, 9,  250.0f, 0, -140.0f, -140.0f, 0, 0x2B, true},
                {"US915 LongFast", 906.875f, 11, 250.0f, 0, -140.0f, -140.0f, 0, 0x2B, true},
                {"US915 MedFast",  906.875f, 9,  250.0f, 0, -140.0f, -140.0f, 0, 0x2B, true},
                {"433 LongFast",   433.175f, 11, 250.0f, 0, -140.0f, -140.0f, 0, 0x2B, true},
                {"AS923 LongFast", 923.000f, 11, 250.0f, 0, -140.0f, -140.0f, 0, 0x2B, true},
                {"AU915 LongFast", 915.000f, 11, 250.0f, 0, -140.0f, -140.0f, 0, 0x2B, true},
            };
        }},
        {"LoRaWAN EU868 Band", [&]() {
            scanCr = 5;
            scanSyncWord = 0x34;
            scanPreambleLen = 8;
            channels = {
                {"EU868.1 Ch1", 868.100f, 7, 125.0f, 0, -140.0f, -140.0f, 0, 0x34, true},
                {"EU868.3 Ch2", 868.300f, 7, 125.0f, 0, -140.0f, -140.0f, 0, 0x34, true},
                {"EU868.5 Ch3", 868.500f, 7, 125.0f, 0, -140.0f, -140.0f, 0, 0x34, true},
                {"EU867.1 Ch4", 867.100f, 7, 125.0f, 0, -140.0f, -140.0f, 0, 0x34, true},
                {"EU867.3 Ch5", 867.300f, 7, 125.0f, 0, -140.0f, -140.0f, 0, 0x34, true},
                {"EU867.5 Ch6", 867.500f, 7, 125.0f, 0, -140.0f, -140.0f, 0, 0x34, true},
                {"EU867.7 Ch7", 867.700f, 7, 125.0f, 0, -140.0f, -140.0f, 0, 0x34, true},
                {"EU867.9 Ch8", 867.900f, 7, 125.0f, 0, -140.0f, -140.0f, 0, 0x34, true},
            };
        }},
        {"LoRaWAN US915 Band", [&]() {
            scanCr = 5;
            scanSyncWord = 0x34;
            scanPreambleLen = 8;
            channels = {
                {"US902.3 Ch1", 902.300f, 7, 125.0f, 0, -140.0f, -140.0f, 0, 0x34, true},
                {"US902.5 Ch2", 902.500f, 7, 125.0f, 0, -140.0f, -140.0f, 0, 0x34, true},
                {"US902.7 Ch3", 902.700f, 7, 125.0f, 0, -140.0f, -140.0f, 0, 0x34, true},
                {"US902.9 Ch4", 902.900f, 7, 125.0f, 0, -140.0f, -140.0f, 0, 0x34, true},
                {"US903.1 Ch5", 903.100f, 7, 125.0f, 0, -140.0f, -140.0f, 0, 0x34, true},
                {"US903.3 Ch6", 903.300f, 7, 125.0f, 0, -140.0f, -140.0f, 0, 0x34, true},
                {"US903.5 Ch7", 903.500f, 7, 125.0f, 0, -140.0f, -140.0f, 0, 0x34, true},
                {"US903.7 Ch8", 903.700f, 7, 125.0f, 0, -140.0f, -140.0f, 0, 0x34, true},
            };
        }},
        {"433 MHz ISM Band", [&]() {
            channels.clear();
            for (float f = 433.050f; f <= 434.750f; f += 0.200f) {
                channels.push_back({String(f, 3) + " MHz", f, 9, 125.0f, 0, -140.0f, -140.0f, 0});
            }
            scanCr = 5;
            scanSyncWord = 0x12;
            scanPreambleLen = 8;
        }},
        {"SF Sweeper (Current Freq)", [&]() {
            channels.clear();
            for (uint8_t sf = 7; sf <= 12; sf++) {
                channels.push_back({"SF" + String(sf) + " (" + String(loraConfig.freqMHz, 2) + "M)",
                                    loraConfig.freqMHz, sf, loraConfig.bwKHz, 0, -140.0f, -140.0f, 0});
            }
        }},
        {"Sync Word Sweep (Current Freq)", [&]() {
            channels = {
                {"Private", loraConfig.freqMHz, loraConfig.sf, loraConfig.bwKHz, 0, -140.0f, -140.0f, 0, 0x12, true},
                {"Meshtastic", loraConfig.freqMHz, loraConfig.sf, loraConfig.bwKHz, 0, -140.0f, -140.0f, 0, 0x2B, true},
                {"LoRaWAN", loraConfig.freqMHz, loraConfig.sf, loraConfig.bwKHz, 0, -140.0f, -140.0f, 0, 0x34, true},
            };
        }}
    };

    int chosen = loopOptions(scanModes, MENU_TYPE_SUBMENU, "Select Scan Band", 0, false, false, 0, true, FP);
    if (chosen < 0 || channels.empty()) return;

    for (auto &channel : channels) {
        if (!channel.hasSyncWord) channel.syncWord = scanSyncWord;
    }

    // Drain any leftover Enter/Esc press from menu selection
    drainKeyboardInput();

    // Start Radio
    LoRaConfigData scanCfg = loraConfig;
    scanCfg.freqMHz = channels[0].freqMHz;
    scanCfg.sf = channels[0].sf;
    scanCfg.bwKHz = channels[0].bwKHz;
    scanCfg.cr = scanCr;
    scanCfg.syncWord = channels[0].syncWord;
    scanCfg.preambleLen = scanPreambleLen;

    if (!initLoRaRadio(scanCfg, true)) {
        displayError("LoRa Radio Init Failed", true);
        return;
    }
    Serial.printf(
        "[LoRaDetector] RX started: %u channels, %.3fMHz SF%d BW%.2fkHz CR4/%d Sync 0x%02X\n",
        (unsigned)channels.size(), scanCfg.freqMHz, scanCfg.sf, scanCfg.bwKHz, scanCfg.cr, scanCfg.syncWord
    );

    int selectedIdx = 0;
    int scrollOffset = 0;
    size_t currentChIdx = 0;
    uint32_t lastHopTime = millis();
    uint32_t lastRssiCheck = 0;
    bool isPaused = false;
    uint8_t rxBuffer[256];

    int lineH = FP * LH + 1;
    int maxLines = (tftHeight - BORDER_PAD_Y - 24) / lineH;
    if (maxLines < 1) maxLines = 1;
    int startY = BORDER_PAD_Y + lineH;

    auto drawStatus = [&]() {
        tft.setTextSize(FP);
        tft.fillRect(7, BORDER_PAD_Y, tftWidth - 14, lineH, bruceConfig.bgColor);
        String status = "Detector: " + String(channels.size()) + " Ch";
        if (isPaused) {
            status += " [PAUSED]";
        } else if (!channels.empty()) {
            status += " [Scan: " + String(channels[currentChIdx].freqMHz, 2) + "M]";
            status += " SW" + formatSyncWord(channels[currentChIdx].syncWord);
        }
        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
        tft.drawString(status, 10, BORDER_PAD_Y);
    };

    auto drawChannelLine = [&](int slotIdx, int chIdx, bool isSel) {
        if (slotIdx < 0 || slotIdx >= maxLines) return;
        int currentY = startY + slotIdx * lineH;
        tft.fillRect(7, currentY, tftWidth - 14, lineH, isSel ? bruceConfig.priColor : bruceConfig.bgColor);

        if (chIdx >= 0 && chIdx < (int)channels.size()) {
            const auto &ch = channels[chIdx];
            if (isSel) {
                tft.setTextColor(bruceConfig.bgColor, bruceConfig.priColor);
            } else {
                tft.setTextColor(
                    (ch.hits > 0 && millis() - ch.lastSeenMs < 3000) ? 0x07E0 : TFT_WHITE,
                    bruceConfig.bgColor
                );
            }

            String line = ch.label;
            if (line.length() > 8) line = line.substring(0, 8);
            while (line.length() < 8) line += " ";
            line += " SW";
            line += formatSyncWord(ch.syncWord);

            line += " H:" + String(ch.hits);
            if (ch.peakRssi > -130.0f) {
                line += " " + String((int)ch.peakRssi) + "dB";
            }

            tft.drawString(line, 10, currentY);

            // Activity mini-bar on right
            int barX = tftWidth - 40;
            int barW = 30;
            int barH = lineH - 2;
            int fillW = map(constrain((int)ch.lastRssi, -130, -50), -130, -50, 0, barW);

            if (!isSel) {
                tft.drawRect(barX, currentY + 1, barW, barH, TFT_DARKGREY);
                if (fillW > 0) {
                    tft.fillRect(barX + 1, currentY + 2, fillW - 2, barH - 2,
                                 (ch.lastRssi > -90) ? TFT_GREEN : TFT_ORANGE);
                }
            }
        }
    };

    auto drawFullUI = [&]() {
        drawStatus();
        for (int i = 0; i < maxLines; i++) {
            int chIdx = scrollOffset + i;
            bool isSel = (chIdx == selectedIdx);
            drawChannelLine(i, chIdx, isSel);
        }
        printCenterFootnote("[UP/DN]Pick [SEL]Lock&Action [ESC]Exit");
    };

    auto updateSelection = [&](int oldIdx, int newIdx) {
        int oldSlot = oldIdx - scrollOffset;
        int newSlot = newIdx - scrollOffset;

        int prevScroll = scrollOffset;
        if (selectedIdx < scrollOffset) scrollOffset = selectedIdx;
        if (selectedIdx >= scrollOffset + maxLines) {
            scrollOffset = selectedIdx - maxLines + 1;
        }

        if (scrollOffset != prevScroll) {
            for (int i = 0; i < maxLines; i++) {
                int chIdx = scrollOffset + i;
                drawChannelLine(i, chIdx, chIdx == selectedIdx);
            }
        } else {
            if (oldSlot >= 0 && oldSlot < maxLines) {
                drawChannelLine(oldSlot, oldIdx, false);
            }
            if (newSlot >= 0 && newSlot < maxLines) {
                drawChannelLine(newSlot, newIdx, true);
            }
        }
    };

    drawMainBorder(true);
    drawFullUI();

    while (true) {
        bool up = false;
        bool down = false;
        bool sel = false;
        bool esc = false;

        if (check(PrevPress)) up = true;
        if (check(UpPress)) up = true;
        if (check(PrevPagePress)) up = true;

        if (check(NextPress)) down = true;
        if (check(DownPress)) down = true;
        if (check(NextPagePress)) down = true;

        if (check(SelPress)) sel = true;
        if (check(EscPress)) esc = true;

#if defined(HAS_ENCODER)
        int encSteps = (int)drainRotarySteps();
        if (encSteps > 0) down = true;
        else if (encSteps < 0) up = true;
#endif

        if (up || down || sel || esc) {
            KeyStroke.Clear();
        } else if (KeyStroke.pressed || !KeyStroke.word.empty()) {
            keyStroke k = _getKeyPress();
            AnyKeyPress = false;
            if (k.enter) sel = true;
            if (k.del) esc = true;
            for (auto ch : k.word) {
                char lowerKey = tolower((char)ch);
                uint8_t uKey = (uint8_t)ch;

                if (lowerKey == '`' || lowerKey == 'q' || uKey == 0x1B) {
                    esc = true;
                } else if (ch == ';' || uKey == 0xDA || lowerKey == 'w' || lowerKey == 'k' || ch == '+' || ch == '=') {
                    up = true;
                } else if (ch == '.' || uKey == 0xD9 || lowerKey == 's' || lowerKey == 'j' || ch == '-' || ch == '_') {
                    down = true;
                } else if (ch == '\n' || ch == '\r' || uKey == 13 || lowerKey == 'e') {
                    sel = true;
                } else if (lowerKey == 'p' || ch == ' ') {
                    isPaused = !isPaused;
                    drawStatus();
                } else if (lowerKey == 'c') {
                    for (auto &c : channels) {
                        c.hits = 0;
                        c.peakRssi = -140.0f;
                        c.lastRssi = -140.0f;
                    }
                    drawFullUI();
                }
            }
        } else {
            AnyKeyPress = false;
        }

        if (esc) break;

        if (up && !channels.empty()) {
            int prevIdx = selectedIdx;
            if (selectedIdx > 0) {
                selectedIdx--;
            } else {
                selectedIdx = (int)channels.size() - 1;
            }
            if (selectedIdx != prevIdx) {
                updateSelection(prevIdx, selectedIdx);
            }
        }
        if (down && !channels.empty()) {
            int prevIdx = selectedIdx;
            if (selectedIdx + 1 < (int)channels.size()) {
                selectedIdx++;
            } else {
                selectedIdx = 0;
            }
            if (selectedIdx != prevIdx) {
                updateSelection(prevIdx, selectedIdx);
            }
        }

        if (sel) {
            // Lock onto selected channel and tune radio to it
            if (selectedIdx >= 0 && selectedIdx < (int)channels.size()) {
                const auto selCh = channels[selectedIdx];
                loraConfig.freqMHz = selCh.freqMHz;
                loraConfig.sf = selCh.sf;
                loraConfig.bwKHz = selCh.bwKHz;
                loraConfig.cr = scanCr;
                loraConfig.syncWord = selCh.syncWord;
                loraConfig.preambleLen = scanPreambleLen;
                saveLoRaConfig();
                stopLoRaRadio();

                // Clear input and wait for key release before opening submenu
                drainKeyboardInput();

                std::vector<Option> actionOpts = {
                    {"Start Sniffer on " + String(selCh.freqMHz, 3) + "MHz", runLoRaSniffer},
                    {"Start Signal Tracker", []() { runLoRaTrackerMenu(); }},
                    {"Apply Freq to Config", []() { displaySuccess("Config Updated"); }},
                    {"Resume Detector", []() {}}
                };
                int a = loopOptions(actionOpts, MENU_TYPE_SUBMENU, "Channel Selected");
                if (a == 0 || a == 1) return;

                // Clear input before returning to detector
                drainKeyboardInput();

                scanCfg.freqMHz = channels[currentChIdx].freqMHz;
                scanCfg.sf = channels[currentChIdx].sf;
                scanCfg.bwKHz = channels[currentChIdx].bwKHz;
                scanCfg.syncWord = channels[currentChIdx].syncWord;
                initLoRaRadio(scanCfg, true);
                drawMainBorder(true);
                drawFullUI();
            }
        }

        // Check for incoming packet on currently tuned channel
        if (checkLoRaPacketAvailable()) {
            float rssi = 0, snr = 0, freqErr = 0;
            size_t pktLen = 0;
            int state = readLoRaRawData(rxBuffer, sizeof(rxBuffer), rssi, snr, freqErr, pktLen);

            if ((state == RADIOLIB_ERR_NONE || state == RADIOLIB_ERR_CRC_MISMATCH) && pktLen > 0) {
                auto &ch = channels[currentChIdx];
                const bool crcOk = state == RADIOLIB_ERR_NONE;
                Serial.printf(
                    "[LoRaDetector] RX channel=%u freq=%.3fMHz SF%d BW%.2fkHz sync=0x%02X crc=%s len=%u RSSI=%.1f SNR=%.1f\n",
                    (unsigned)currentChIdx, ch.freqMHz, ch.sf, ch.bwKHz, ch.syncWord, crcOk ? "OK" : "FAIL",
                    (unsigned)pktLen, rssi, snr
                );
                ch.hits++;
                ch.lastSeenMs = millis();
                ch.lastRssi = rssi;
                if (rssi > ch.peakRssi) ch.peakRssi = rssi;

                // Redraw line if visible
                int slot = (int)currentChIdx - scrollOffset;
                if (slot >= 0 && slot < maxLines) {
                    drawChannelLine(slot, currentChIdx, (int)currentChIdx == selectedIdx);
                }

                // Feed packet into global sniffer/node records
                LoRaPacket pkt;
                pkt.timestampMs = millis();
                pkt.freqMHz = ch.freqMHz;
                pkt.sf = ch.sf;
                pkt.bwKHz = ch.bwKHz;
                pkt.cr = scanCfg.cr;
                pkt.syncWord = ch.syncWord;
                pkt.rssi = rssi;
                pkt.snr = snr;
                pkt.freqErrorHz = freqErr;
                pkt.timeOnAirMs = getLoRaTimeOnAir(pktLen);
                pkt.crcOk = crcOk;
                pkt.raw.assign(rxBuffer, rxBuffer + pktLen);

                parseLoRaPacket(pkt);
                addPacketToSniffer(pkt);
            } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
                auto &ch = channels[currentChIdx];
                ch.hits++;
                ch.lastSeenMs = millis();
                ch.lastRssi = getLoRaInstantRSSI();
                if (ch.lastRssi > ch.peakRssi) ch.peakRssi = ch.lastRssi;
                const int slot = (int)currentChIdx - scrollOffset;
                if (slot >= 0 && slot < maxLines) {
                    drawChannelLine(slot, currentChIdx, (int)currentChIdx == selectedIdx);
                }
                Serial.printf(
                    "[LoRaDetector] RX header/CRC error channel=%u freq=%.3fMHz RSSI=%.1f\n",
                    (unsigned)currentChIdx, ch.freqMHz, ch.lastRssi
                );
            } else if (state != -1) {
                Serial.printf("[LoRaDetector] RX read error=%d len=%u\n", state, (unsigned)pktLen);
            }
        }

        // Sample current channel RSSI when paused
        if (isPaused && (millis() - lastRssiCheck >= 300)) {
            lastRssiCheck = millis();
            float instantRssi = getLoRaInstantRSSI();
            if (instantRssi > -135.0f && instantRssi < 0.0f) {
                float prevRssi = channels[currentChIdx].lastRssi;
                if (fabs(instantRssi - prevRssi) >= 3.0f) {
                    channels[currentChIdx].lastRssi = instantRssi;
                    int slot = (int)currentChIdx - scrollOffset;
                    if (slot >= 0 && slot < maxLines) {
                        drawChannelLine(slot, currentChIdx, (int)currentChIdx == selectedIdx);
                    }
                }
            }
        }

        // Dwell on current channel before hopping to next
        if (!isPaused && channels.size() > 1 && (millis() - lastHopTime >= LORA_SCAN_DWELL_MS)) {
            lastHopTime = millis();
            currentChIdx = (currentChIdx + 1) % channels.size();
            auto &ch = channels[currentChIdx];
            setLoRaFrequency(ch.freqMHz);
            setLoRaSpreadingFactor(ch.sf);
            setLoRaBandwidth(ch.bwKHz);
            setLoRaSyncWord(ch.syncWord);
            startLoRaReceive();
            drawStatus();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

exit_detector:
    stopLoRaRadio();
}

#endif // !LITE_VERSION
