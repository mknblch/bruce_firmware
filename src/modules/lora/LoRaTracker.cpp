#if !defined(LITE_VERSION)
#include "LoRaTracker.h"
#include "LoRaConfig.h"
#include "LoRaPacket.h"
#include "LoRaRadio.h"
#include "LoRaSniffer.h"
#include "core/display.h"
#include "core/imu.h"
#include "core/mykeyboard.h"
#include "core/utils.h"
#include <Arduino.h>
#include <math.h>

namespace {

constexpr int HEADING_BUCKETS = 16;
constexpr float HEADING_BUCKET_DEG = 360.0f / HEADING_BUCKETS;
constexpr float HEADING_NOISE_FLOOR = -135.0f;
constexpr float HEADING_BEST_DECAY_DB_PER_SEC = 2.0f;

enum HeadingConfidence {
    CONFIDENCE_LOW = 0,
    CONFIDENCE_MED = 1,
    CONFIDENCE_HIGH = 2
};

struct BestHeadingTable {
    float binRssi[HEADING_BUCKETS];
    uint16_t sampleCount[HEADING_BUCKETS];
    unsigned long lastDecayMs = 0;
    float lastTargetDeg = -1.0f;
    bool hasTarget = false;

    void reset() {
        for (int i = 0; i < HEADING_BUCKETS; i++) {
            binRssi[i] = HEADING_NOISE_FLOOR;
            sampleCount[i] = 0;
        }
        lastDecayMs = millis();
        lastTargetDeg = -1.0f;
        hasTarget = false;
    }

    void decay() {
        unsigned long now = millis();
        float dtSec = (now - lastDecayMs) / 1000.0f;
        lastDecayMs = now;
        if (dtSec <= 0.0f) return;
        float drop = HEADING_BEST_DECAY_DB_PER_SEC * dtSec;
        for (int i = 0; i < HEADING_BUCKETS; i++) {
            if (binRssi[i] > HEADING_NOISE_FLOOR) {
                binRssi[i] -= drop;
                if (binRssi[i] < HEADING_NOISE_FLOOR) {
                    binRssi[i] = HEADING_NOISE_FLOOR;
                    sampleCount[i] = 0;
                }
            }
        }
    }

    void feed(uint16_t bucket, int8_t rssi) {
        if (bucket >= HEADING_BUCKETS) return;
        float r = (float)rssi;
        if (r < HEADING_NOISE_FLOOR) r = HEADING_NOISE_FLOOR;

        if (sampleCount[bucket] == 0 || binRssi[bucket] <= HEADING_NOISE_FLOOR) {
            binRssi[bucket] = r;
            sampleCount[bucket] = 1;
        } else {
            binRssi[bucket] = 0.35f * r + 0.65f * binRssi[bucket];
            if (sampleCount[bucket] < 100) sampleCount[bucket]++;
        }
    }

    float getTargetBearingDeg(bool &resolved, HeadingConfidence &conf) {
        decay();
        float sumX = 0.0f;
        float sumY = 0.0f;
        float totalWeight = 0.0f;
        int totalSamples = 0;
        int populatedSectors = 0;

        for (int i = 0; i < HEADING_BUCKETS; i++) {
            if (binRssi[i] > HEADING_NOISE_FLOOR && sampleCount[i] > 0) {
                totalSamples += sampleCount[i];
                populatedSectors++;

                float sig = binRssi[i] - HEADING_NOISE_FLOOR;
                float countFactor = min((float)sampleCount[i], 4.0f) / 4.0f;
                float w = (sig * sig) * (0.4f + 0.6f * countFactor);

                float rad = (i * HEADING_BUCKET_DEG) * (M_PI / 180.0f);
                sumX += w * cosf(rad);
                sumY += w * sinf(rad);
                totalWeight += w;
            }
        }

        float vectorMag = sqrtf(sumX * sumX + sumY * sumY);
        float directivity = (totalWeight > 0.001f) ? (vectorMag / totalWeight) : 0.0f;

        if (totalSamples < 4 || populatedSectors < 2) {
            conf = CONFIDENCE_LOW;
            resolved = false;
            return 0.0f;
        } else if (totalSamples < 8 || populatedSectors < 4 || directivity < 0.25f) {
            conf = CONFIDENCE_MED;
        } else {
            conf = CONFIDENCE_HIGH;
        }

        if (totalWeight > 4.0f && vectorMag > 2.0f) {
            float angleRad = atan2f(sumY, sumX);
            float targetDeg = angleRad * (180.0f / M_PI);
            targetDeg = fmodf(targetDeg + 360.0f, 360.0f);

            if (!hasTarget) {
                lastTargetDeg = targetDeg;
                hasTarget = true;
            } else {
                float diff = targetDeg - lastTargetDeg;
                while (diff > 180.0f) diff -= 360.0f;
                while (diff < -180.0f) diff += 360.0f;
                lastTargetDeg = fmodf(lastTargetDeg + 0.25f * diff + 360.0f, 360.0f);
            }
            resolved = true;
            return lastTargetDeg;
        }

        resolved = hasTarget;
        return hasTarget ? lastTargetDeg : 0.0f;
    }
};

static uint16_t headingDegToBucket(float deg) {
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    int bucket = (int)((deg + HEADING_BUCKET_DEG / 2.0f) / HEADING_BUCKET_DEG);
    return (uint16_t)(bucket % HEADING_BUCKETS);
}

void drawHeadingArrow(int cx, int cy, int radius, float angleDeg, uint16_t color) {
    float rad = angleDeg * (PI / 180.0f);
    float dx = sinf(rad);
    float dy = -cosf(rad);

    int tipX = cx + (int)(dx * radius);
    int tipY = cy + (int)(dy * radius);

    float backSpread = 135.0f * (PI / 180.0f);
    int backLen = radius / 2;
    int leftX = cx + (int)(sinf(rad + backSpread) * backLen);
    int leftY = cy + (int)(-cosf(rad + backSpread) * backLen);
    int rightX = cx + (int)(sinf(rad - backSpread) * backLen);
    int rightY = cy + (int)(-cosf(rad - backSpread) * backLen);

    tft.fillTriangle(tipX, tipY, leftX, leftY, rightX, rightY, color);
}

uint16_t getVuColor(float f) {
    f = constrain(f, 0.0f, 1.0f);
    uint8_t r = 0, g = 0, b = 0;
    if (f < 0.5f) {
        r = (uint8_t)(255.0f * (f / 0.5f));
        g = 255;
        b = 0;
    } else {
        r = 255;
        g = (uint8_t)(255.0f * (1.0f - (f - 0.5f) / 0.5f));
        b = 0;
    }
    return tft.color565(r, g, b);
}

} // namespace

void trackLoRaTarget(const String &targetMacOrId, const String &label) {
    loadLoRaConfig();
    if (!isLoraHardwareConfigured()) {
        displayError("LoRa pins not configured!", true);
        return;
    }

    displayTextLine("Init LoRa Tracker...");

    if (!initLoRaRadio(loraConfig, true)) {
        displayError("LoRa Radio Init Failed", true);
        return;
    }

    String displayLabel = label.isEmpty() ? (targetMacOrId.isEmpty() ? "Any Transmitter" : targetMacOrId) : label;
    bool trackAny = targetMacOrId.isEmpty() || targetMacOrId.equalsIgnoreCase("ANY");

    bool hasImu = imu_available();
    if (hasImu) {
        drawMainBorder();
        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
        tft.drawCentreString("IMU Calibration", tftWidth / 2, BORDER_PAD_Y, 1);
        tft.drawCentreString("Hold device still...", tftWidth / 2, tftHeight / 2 - 16, 1);

        int calBarW = tftWidth - 2 * BORDER_PAD_X - 20;
        int calBarH = 10;
        int calBarX = (tftWidth - calBarW) / 2;
        int calBarY = tftHeight / 2 + 8;
        tft.drawRect(calBarX, calBarY, calBarW, calBarH, TFT_DARKGREY);

        imu_calibrate(600, [=](int pct) {
            int fillW = (calBarW - 4) * pct / 100;
            if (fillW > 0) {
                tft.fillRect(calBarX + 2, calBarY + 2, fillW, calBarH - 4, bruceConfig.priColor);
            }
        });
    }

    BestHeadingTable bestHeading;
    if (hasImu) bestHeading.reset();

    tft.fillScreen(bruceConfig.bgColor);
    drawMainBorder(true);

    int barX = BORDER_PAD_X;
    int barY = BORDER_PAD_Y + FM * LH + 2;
    int barW = tftWidth - 2 * BORDER_PAD_X;
    int barH = 12;
    int readoutY = barY + barH + 4;

    constexpr int segW = 4;
    constexpr int segGap = 2;
    int numSegments = max(1, (barW - 4) / (segW + segGap));
    int actualBarW = numSegments * (segW + segGap) - segGap + 4;
    int actualBarX = barX + (barW - actualBarW) / 2;

    int arrowCenterX = BORDER_PAD_X + 22;
    int arrowCenterY = readoutY + LH * FP + 22;
    int arrowRadius = 16;
    if (arrowCenterY + arrowRadius + 4 > tftHeight) {
        arrowRadius = max(10, (tftHeight - 4 - (readoutY + LH * FP + 4)) / 2);
        arrowCenterY = (readoutY + LH * FP + 4) + arrowRadius + 2;
    }
    int infoTextX = arrowCenterX + arrowRadius + 10;

    float emaRssi = -130.0f;
    bool emaInitialized = false;
    unsigned long lastSeenMs = 0;
    float peakFraction = 0.0f;
    uint32_t packetCount = 0;
    float currentSnr = 0;
    float currentHeadingDeg = 0.0f;
    uint8_t rxBuffer[256];
    uint32_t lastUiDraw = 0;

    // Drain any leftover Enter/Esc press from previous menu
    vTaskDelay(pdMS_TO_TICKS(150));
    check(SelPress);
    check(EscPress);

    while (true) {
        if (check(EscPress)) break;

        keyStroke k = _getKeyPress();
        if (k.pressed || !k.word.empty()) {
            if (k.del || k.exit_key) {
                break;
            }
            for (auto ch : k.word) {
                char lowerKey = tolower(ch);
                if (lowerKey == '`') {
                    goto exit_tracker;
                } else if (lowerKey == 'c') {
                    packetCount = 0;
                    emaInitialized = false;
                    peakFraction = 0.0f;
                    if (hasImu) bestHeading.reset();
                }
            }
        }

        if (hasImu) {
            currentHeadingDeg = imu_get_heading_delta_deg();
            bestHeading.decay();
        }

        // Receive Packet Check
        if (checkLoRaPacketAvailable()) {
            float rssi = 0, snr = 0, freqErr = 0;
            size_t pktLen = 0;
            int state = readLoRaRawData(rxBuffer, sizeof(rxBuffer), rssi, snr, freqErr, pktLen);

            if (state == RADIOLIB_ERR_NONE && pktLen > 0) {
                LoRaPacket pkt;
                pkt.timestampMs = millis();
                pkt.freqMHz = loraConfig.freqMHz;
                pkt.sf = loraConfig.sf;
                pkt.bwKHz = loraConfig.bwKHz;
                pkt.syncWord = loraConfig.syncWord;
                pkt.rssi = rssi;
                pkt.snr = snr;
                pkt.raw.assign(rxBuffer, rxBuffer + pktLen);
                parseLoRaPacket(pkt);

                bool matches = trackAny;
                if (!matches) {
                    if (pkt.sender.indexOf(targetMacOrId) >= 0 ||
                        pkt.destination.indexOf(targetMacOrId) >= 0 ||
                        pkt.payloadAscii.indexOf(targetMacOrId) >= 0) {
                        matches = true;
                    }
                }

                if (matches) {
                    packetCount++;
                    currentSnr = snr;
                    lastSeenMs = millis();

                    constexpr float EMA_ALPHA = 0.35f;
                    emaRssi = emaInitialized ? (EMA_ALPHA * rssi + (1.0f - EMA_ALPHA) * emaRssi) : rssi;
                    emaInitialized = true;

                    float rawFraction = constrain((rssi - (-130.0f)) / 75.0f, 0.0f, 1.0f);
                    if (rawFraction >= peakFraction) {
                        peakFraction = rawFraction;
                    }

                    if (hasImu) {
                        uint16_t bucket = headingDegToBucket(currentHeadingDeg);
                        bestHeading.feed(bucket, (int8_t)rssi);
                    }
                }
            }
        }

        // Decay peak bar over time
        peakFraction = max(0.0f, peakFraction - 0.015f);

        // Render Frame
        if (millis() - lastUiDraw > 100) {
            lastUiDraw = millis();
            bool stale = !emaInitialized || (millis() - lastSeenMs > 5000);

            // Top Header
            tft.setTextSize(FP);
            tft.fillRect(BORDER_PAD_X, BORDER_PAD_Y, tftWidth - 2 * BORDER_PAD_X, FM * LH, bruceConfig.bgColor);
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            String headStr = displayLabel;
            if (headStr.length() > 16) headStr = headStr.substring(0, 16);
            tft.drawString(headStr, BORDER_PAD_X, BORDER_PAD_Y);

            String badge = "Pkts:" + String(packetCount);
            tft.drawString(badge, tftWidth - BORDER_PAD_X - (badge.length() * LW * FP), BORDER_PAD_Y);

            // VU RSSI Meter Bar
            tft.drawRect(actualBarX, barY, actualBarW, barH, TFT_DARKGREY);
            float currentFraction = (!stale && emaInitialized) ? constrain((emaRssi - (-130.0f)) / 75.0f, 0.0f, 1.0f) : 0.0f;
            int filledSegments = (int)(currentFraction * numSegments);
            int peakSegment = (int)(peakFraction * numSegments);

            for (int s = 0; s < numSegments; s++) {
                int segX = actualBarX + 2 + s * (segW + segGap);
                int segY = barY + 2;
                int segH = barH - 4;
                float segFrac = (float)s / (float)numSegments;
                uint16_t segCol = getVuColor(segFrac);

                if (!stale && s < filledSegments) {
                    tft.fillRect(segX, segY, segW, segH, segCol);
                } else if (!stale && s == peakSegment && peakSegment > 0) {
                    tft.fillRect(segX, segY, segW, segH, TFT_WHITE);
                } else {
                    tft.fillRect(segX, segY, segW, segH, TFT_BLACK);
                }
            }

            // Readout Line
            tft.fillRect(BORDER_PAD_X, readoutY, tftWidth - 2 * BORDER_PAD_X, LH * FP + 2, bruceConfig.bgColor);
            if (!stale && emaInitialized) {
                String rssiStr = String(emaRssi, 1) + " dBm (SNR: " + String(currentSnr, 1) + "dB)";
                tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
                tft.drawString(rssiStr, BORDER_PAD_X, readoutY);
            } else {
                tft.setTextColor(TFT_DARKGREY, bruceConfig.bgColor);
                tft.drawString(emaInitialized ? "Signal stale..." : "Listening for signal...", BORDER_PAD_X, readoutY);
            }

            // Direction & Compass Section (IMU)
            if (hasImu) {
                bool resolved = false;
                HeadingConfidence conf = CONFIDENCE_LOW;
                float targetDeg = bestHeading.getTargetBearingDeg(resolved, conf);

                tft.fillRect(arrowCenterX - arrowRadius - 2, arrowCenterY - arrowRadius - 2,
                             (arrowRadius + 2) * 2, (arrowRadius + 2) * 2, bruceConfig.bgColor);
                tft.drawCircle(arrowCenterX, arrowCenterY, arrowRadius, TFT_DARKGREY);
                tft.drawPixel(arrowCenterX, arrowCenterY - arrowRadius, TFT_RED); // North mark

                if (resolved && !stale) {
                    float relativeAngle = fmodf(targetDeg - currentHeadingDeg + 360.0f, 360.0f);
                    uint16_t arrowColor = (conf == CONFIDENCE_HIGH) ? TFT_GREEN : (conf == CONFIDENCE_MED ? TFT_YELLOW : TFT_RED);
                    drawHeadingArrow(arrowCenterX, arrowCenterY, arrowRadius - 2, relativeAngle, arrowColor);
                }

                int textY = readoutY + LH * FP + 6;
                tft.fillRect(infoTextX, textY, tftWidth - infoTextX - BORDER_PAD_X, 3 * LH * FP + 4, bruceConfig.bgColor);

                tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
                tft.drawString("Bearing: " + (resolved ? (String((int)targetDeg) + " deg") : "Sweeping..."), infoTextX, textY);

                tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
                unsigned long ago = (lastSeenMs > 0) ? (millis() - lastSeenMs) / 1000 : 999;
                tft.drawString("Last pkt: " + (lastSeenMs > 0 ? (String(ago) + "s ago") : "never"), infoTextX, textY + LH * FP);

                String qual = (!stale && emaRssi > -80.0f) ? "STRONG" : ((!stale && emaRssi > -105.0f) ? "GOOD" : ((!stale && emaRssi > -120.0f) ? "FAIR" : "NO SIG"));
                tft.drawString("Signal: " + qual, infoTextX, textY + 2 * LH * FP);
            } else {
                // Non-IMU layout
                int textY = readoutY + LH * FP + 6;
                tft.fillRect(BORDER_PAD_X, textY, tftWidth - 2 * BORDER_PAD_X, 3 * LH * FP + 4, bruceConfig.bgColor);

                unsigned long ago = (lastSeenMs > 0) ? (millis() - lastSeenMs) / 1000 : 999;
                tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
                tft.drawString("Last pkt: " + (lastSeenMs > 0 ? (String(ago) + "s ago") : "never"), BORDER_PAD_X, textY);

                String qual = (!stale && emaRssi > -80.0f) ? "STRONG" : ((!stale && emaRssi > -105.0f) ? "GOOD" : ((!stale && emaRssi > -120.0f) ? "FAIR" : "NO SIG"));
                tft.drawString("Link Quality: " + qual, BORDER_PAD_X, textY + LH * FP);
                tft.drawString("Freq: " + String(loraConfig.freqMHz, 3) + " MHz (SF" + String(loraConfig.sf) + ")", BORDER_PAD_X, textY + 2 * LH * FP);
            }

            printCenterFootnote("[C]Clear [ESC]Exit");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

exit_tracker:
    stopLoRaRadio();
}

static void liveScanAndPickLoRaTarget() {
    displayTextLine("Scanning LoRa signals (5s)...");
    if (!initLoRaRadio(loraConfig, true)) {
        displayError("Radio Init Failed", true);
        return;
    }

    std::vector<LoRaNodeRecord> localNodes = gLoRaNodes;
    uint32_t startMs = millis();
    uint8_t rxBuffer[256];

    while (millis() - startMs < 5000) {
        if (check(EscPress)) break;

        if (checkLoRaPacketAvailable()) {
            float rssi = 0, snr = 0, freqErr = 0;
            size_t pktLen = 0;
            int state = readLoRaRawData(rxBuffer, sizeof(rxBuffer), rssi, snr, freqErr, pktLen);

            if (state == RADIOLIB_ERR_NONE && pktLen > 0) {
                LoRaPacket pkt;
                pkt.timestampMs = millis();
                pkt.freqMHz = loraConfig.freqMHz;
                pkt.sf = loraConfig.sf;
                pkt.bwKHz = loraConfig.bwKHz;
                pkt.syncWord = loraConfig.syncWord;
                pkt.rssi = rssi;
                pkt.snr = snr;
                pkt.raw.assign(rxBuffer, rxBuffer + pktLen);
                parseLoRaPacket(pkt);
                addPacketToSniffer(pkt);

                String addr = pkt.sender.length() > 0 ? pkt.sender : ("RAW_" + String(pkt.freqMHz, 2) + "M");
                bool found = false;
                for (auto &n : localNodes) {
                    if (n.address == addr) {
                        n.packetCount++;
                        n.lastRssi = rssi;
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    LoRaNodeRecord n;
                    n.address = addr;
                    n.displayName = addr;
                    n.protocol = pkt.protocolName;
                    n.lastRssi = rssi;
                    n.packetCount = 1;
                    localNodes.push_back(n);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    stopLoRaRadio();

    if (localNodes.empty()) {
        displayError("No signals detected", true);
        return;
    }

    std::vector<Option> pickOpts;
    for (const auto &n : localNodes) {
        String title = n.address + " [" + n.protocol + "] " + String((int)n.lastRssi) + "dBm (" + String(n.packetCount) + "pkts)";
        pickOpts.push_back({title, [=]() {
            trackLoRaTarget(n.address, n.displayName);
        }});
    }

    loopOptions(pickOpts, MENU_TYPE_SUBMENU, "Select Target");
}

void runLoRaTrackerMenu() {
    loadLoRaConfig();
    if (!isLoraHardwareConfigured()) {
        displayError("LoRa pins not configured!", true);
        return;
    }

    std::vector<Option> options;

    options.push_back({"Live Scan & Pick Target", liveScanAndPickLoRaTarget});

    if (!gLoRaNodes.empty()) {
        options.push_back({"Pick from Active Nodes (" + String(gLoRaNodes.size()) + ")", []() {
            std::vector<Option> nodeOpts;
            for (size_t i = 0; i < gLoRaNodes.size(); i++) {
                const auto &n = gLoRaNodes[i];
                String title = n.address + " [" + n.protocol + "] (" + String((int)n.lastRssi) + "dBm)";
                nodeOpts.push_back({title, [=]() {
                    trackLoRaTarget(n.address, n.displayName);
                }});
            }
            loopOptions(nodeOpts, MENU_TYPE_SUBMENU, "Select Target");
        }});
    }

    options.push_back({"Track Any Transmitter", []() {
        trackLoRaTarget("", "Any Transmitter");
    }});

    options.push_back({"Enter Target Node/DevAddr", []() {
        tft.fillScreen(bruceConfig.bgColor);
        String target = keyboard("", 24, "Target (e.g. !1a2b3c4d):");
        if (target != "" && target != "\x1B") {
            trackLoRaTarget(target, target);
        }
    }});

    loopOptions(options, MENU_TYPE_SUBMENU, "LoRa Tracker");
}

#endif // !LITE_VERSION
