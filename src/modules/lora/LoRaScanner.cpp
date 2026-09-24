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
};

void runLoRaChannelDetector() {
    loadLoRaConfig();
    if (!isLoraHardwareConfigured()) {
        displayError("LoRa pins not configured!", true);
        return;
    }

    displayTextLine("Init Channel Detector...");

    std::vector<ScanChannelEntry> channels;

    // Pick scan mode
    std::vector<Option> scanModes = {
        {"Meshtastic Presets", [&]() {
            channels = {
                {"EU868 LongFast", 869.525f, 11, 250.0f, 0, -140.0f, -140.0f, 0},
                {"EU868 MedFast",  869.525f, 9,  250.0f, 0, -140.0f, -140.0f, 0},
                {"US915 LongFast", 906.875f, 11, 250.0f, 0, -140.0f, -140.0f, 0},
                {"US915 MedFast",  906.875f, 9,  250.0f, 0, -140.0f, -140.0f, 0},
                {"433 LongFast",   433.175f, 11, 250.0f, 0, -140.0f, -140.0f, 0},
                {"AS923 LongFast", 923.000f, 11, 250.0f, 0, -140.0f, -140.0f, 0},
                {"AU915 LongFast", 915.000f, 11, 250.0f, 0, -140.0f, -140.0f, 0},
            };
        }},
        {"LoRaWAN EU868 Band", [&]() {
            channels = {
                {"EU868.1 Ch1", 868.100f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
                {"EU868.3 Ch2", 868.300f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
                {"EU868.5 Ch3", 868.500f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
                {"EU867.1 Ch4", 867.100f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
                {"EU867.3 Ch5", 867.300f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
                {"EU867.5 Ch6", 867.500f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
                {"EU867.7 Ch7", 867.700f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
                {"EU867.9 Ch8", 867.900f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
            };
        }},
        {"LoRaWAN US915 Band", [&]() {
            channels = {
                {"US902.3 Ch1", 902.300f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
                {"US902.5 Ch2", 902.500f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
                {"US902.7 Ch3", 902.700f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
                {"US902.9 Ch4", 902.900f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
                {"US903.1 Ch5", 903.100f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
                {"US903.3 Ch6", 903.300f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
                {"US903.5 Ch7", 903.500f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
                {"US903.7 Ch8", 903.700f, 7, 125.0f, 0, -140.0f, -140.0f, 0},
            };
        }},
        {"433 MHz ISM Band", [&]() {
            channels.clear();
            for (float f = 433.050f; f <= 434.750f; f += 0.200f) {
                channels.push_back({String(f, 3) + " MHz", f, 9, 125.0f, 0, -140.0f, -140.0f, 0});
            }
        }},
        {"SF Sweeper (Current Freq)", [&]() {
            channels.clear();
            for (uint8_t sf = 7; sf <= 12; sf++) {
                channels.push_back({"SF" + String(sf) + " (" + String(loraConfig.freqMHz, 2) + "M)",
                                    loraConfig.freqMHz, sf, loraConfig.bwKHz, 0, -140.0f, -140.0f, 0});
            }
        }}
    };

    int chosen = loopOptions(scanModes, MENU_TYPE_SUBMENU, "Select Scan Band");
    if (chosen < 0 || channels.empty()) return;

    // Start Radio
    LoRaConfigData scanCfg = loraConfig;
    scanCfg.freqMHz = channels[0].freqMHz;
    scanCfg.sf = channels[0].sf;
    scanCfg.bwKHz = channels[0].bwKHz;

    if (!initLoRaRadio(scanCfg, true)) {
        displayError("LoRa Radio Init Failed", true);
        return;
    }

    int selectedIdx = 0;
    int scrollOffset = 0;
    size_t currentChIdx = 0;
    uint32_t lastUiUpdate = 0;
    uint32_t lastCadStep = 0;
    bool isPaused = false;
    bool needsRedraw = true;

    drawMainBorder(true);

    while (true) {
        if (check(EscPress)) break;

        if (check(PrevPress) || check(UpPress)) {
            if (selectedIdx > 0) {
                selectedIdx--;
                needsRedraw = true;
            }
        }
        if (check(NextPress) || check(DownPress)) {
            if (selectedIdx + 1 < (int)channels.size()) {
                selectedIdx++;
                needsRedraw = true;
            }
        }

        if (check(SelPress)) {
            // Lock onto selected channel and tune radio to it
            if (selectedIdx >= 0 && selectedIdx < (int)channels.size()) {
                const auto selCh = channels[selectedIdx];
                loraConfig.freqMHz = selCh.freqMHz;
                loraConfig.sf = selCh.sf;
                loraConfig.bwKHz = selCh.bwKHz;
                saveLoRaConfig();
                stopLoRaRadio();

                std::vector<Option> actionOpts = {
                    {"Start Sniffer on " + String(selCh.freqMHz, 3) + "MHz", runLoRaSniffer},
                    {"Start Signal Tracker", []() { runLoRaTrackerMenu(); }},
                    {"Apply Freq to Config", []() { displaySuccess("Config Updated"); }},
                    {"Resume Detector", []() {}}
                };
                int a = loopOptions(actionOpts, MENU_TYPE_SUBMENU, "Channel Selected");
                if (a == 0 || a == 1) return;

                // Resume Detector
                initLoRaRadio(scanCfg, true);
                drawMainBorder(true);
                needsRedraw = true;
            }
        }

        char key = checkLetterShortcutPress();
        if (key == 'p' || key == 'P') {
            isPaused = !isPaused;
            needsRedraw = true;
        } else if (key == 'c' || key == 'C') {
            for (auto &ch : channels) {
                ch.hits = 0;
                ch.peakRssi = -140.0f;
                ch.lastRssi = -140.0f;
            }
            needsRedraw = true;
        }

        // Cycle through channels & do CAD detection
        if (!isPaused && !channels.empty() && (millis() - lastCadStep >= 40)) {
            lastCadStep = millis();
            auto &ch = channels[currentChIdx];
            setLoRaFrequency(ch.freqMHz);
            setLoRaSpreadingFactor(ch.sf);
            setLoRaBandwidth(ch.bwKHz);

            int cadResult = scanLoRaCAD();
            float rssi = getLoRaInstantRSSI();
            ch.lastRssi = rssi;

            if (cadResult == RADIOLIB_PREAMBLE_DETECTED || rssi > -105.0f) {
                ch.hits++;
                ch.lastSeenMs = millis();
                if (rssi > ch.peakRssi) ch.peakRssi = rssi;
                needsRedraw = true;
            }

            currentChIdx = (currentChIdx + 1) % channels.size();
        }

        // Render UI
        if (millis() - lastUiUpdate > 150 || needsRedraw) {
            lastUiUpdate = millis();
            needsRedraw = false;

            int lineH = FP * LH + 1;
            int maxLines = (tftHeight - BORDER_PAD_Y - 24) / lineH;
            if (maxLines < 1) maxLines = 1;
            int startY = BORDER_PAD_Y + lineH;

            // Status bar
            tft.setTextSize(FP);
            tft.fillRect(7, BORDER_PAD_Y, tftWidth - 14, lineH, bruceConfig.bgColor);
            String status = "CAD Detector: " + String(channels.size()) + " Ch";
            if (isPaused) status += " [PAUSED]";
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString(status, 10, BORDER_PAD_Y);

            if (selectedIdx < scrollOffset) scrollOffset = selectedIdx;
            if (selectedIdx >= scrollOffset + maxLines) {
                scrollOffset = selectedIdx - maxLines + 1;
            }

            for (int i = 0; i < maxLines; i++) {
                int chIdx = scrollOffset + i;
                int currentY = startY + i * lineH;
                tft.fillRect(7, currentY, tftWidth - 14, lineH, bruceConfig.bgColor);

                if (chIdx < (int)channels.size()) {
                    const auto &ch = channels[chIdx];
                    bool isSel = (chIdx == selectedIdx);

                    if (isSel) {
                        tft.fillRect(7, currentY, tftWidth - 14, lineH, bruceConfig.priColor);
                        tft.setTextColor(bruceConfig.bgColor, bruceConfig.priColor);
                    } else {
                        tft.setTextColor(
                            (ch.hits > 0 && millis() - ch.lastSeenMs < 3000) ? 0x07E0 : TFT_WHITE,
                            bruceConfig.bgColor
                        );
                    }

                    String line = ch.label;
                    if (line.length() > 14) line = line.substring(0, 14);
                    while (line.length() < 14) line += " ";

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
            }

            printCenterFootnote("[UP/DN]Pick [SEL]Lock&Action [ESC]Exit");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    stopLoRaRadio();
}

#endif // !LITE_VERSION
