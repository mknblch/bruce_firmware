#include "ble_tracker.h"
#include "core/bus_HAL.h"
#include "core/config.h"
#include "core/display.h"
#include "core/imu.h"
#include "core/mykeyboard.h"
#include "core/utils.h"
#include "modules/ble/ble_common.h"
#if !defined(LITE_VERSION)
#include "BLE_Suite.h"
#include "modules/ble/gatt_explorer.h"
#endif
#include <math.h>

namespace {

#if !defined(LITE_VERSION)
// Reuses the GATT Explorer's live scan + picker screen (device count, RSSI bars, minRSSI /
// connectable / public-random settings, cancelable with ESC/SEL) instead of a bespoke
// passive-only scanner - picking a device locks onto its MAC instead of entering GATT
// service exploration.
void bleTrackerPickFromScan() {
    gattScanAndPick([](const String &name, const String &mac, int rssi, uint8_t addrType) {
        bleTrackerLockTarget(name, mac);
    });
}
#else
// LITE_VERSION has no GATT Explorer to reuse, so fall back to a plain passive
// (setActiveScan(false)) one-shot scan rendered as a pick-list, mirroring ble_scan()'s
// options/loopOptions() flow in ble_common.cpp - no scan requests are ever sent, and picking
// an entry locks onto its MAC instead of opening the read-only "info" screen.
void bleTrackerPickFromScan() {
    displayTextLine("Scanning (passive)..");

    options = {};
    options.reserve(MAX_DISPLAY_DEVICES);

    bool bleWasActiveBefore = BLEConnected || (BLEDevice::getServer() != nullptr);

    if (!ble_scan_setup() || pBLEScan == nullptr) {
        displayError("Failed to init BLE scan");
        return;
    }

    pBLEScan->setActiveScan(false); // passive: never send scan requests
    pBLEScan->clearResults();

    try {
        BLEScanResults foundDevices = pBLEScan->getResults(scanTime * 1000, false);
        int deviceCount = foundDevices.getCount();
        int maxToProcess = min(deviceCount, MAX_DISPLAY_DEVICES);

        for (int i = 0; i < maxToProcess; i++) {
            const NimBLEAdvertisedDevice *advertisedDevice = foundDevices.getDevice(i);
            if (!advertisedDevice) continue;

            String name = advertisedDevice->getName().c_str();
            String mac = advertisedDevice->getAddress().toString().c_str();
            String rssi = String(advertisedDevice->getRSSI());
            String title = (name.isEmpty() ? mac : name) + " (" + rssi + "dBm)";

            options.emplace_back(title.c_str(), [=]() { bleTrackerLockTarget(name.isEmpty() ? mac : name, mac); });
        }

        if (options.size() >= MAX_DISPLAY_DEVICES) { options.emplace_back("... and more devices", nullptr); }
    } catch (...) {
        displayError("BLE scan error");
        if (pBLEScan) pBLEScan->clearResults();
        return;
    }

    if (pBLEScan) pBLEScan->stop();

    if (!bleWasActiveBefore) { stopBLEStack(); }

    if (options.empty()) {
        displayError("No devices found");
        delay(1000);
        return;
    }

    addOptionToMainMenu();
    loopOptions(options);
    options.clear();
}
#endif

// Free-form label prompt reused by both the "save as favorite" flow below and (in a later
// revision) the favorites list itself.
String promptForLabel(const String &defaultLabel) {
    String label = keyboard(defaultLabel, 24, "Label for favorite");
    return label.isEmpty() ? defaultLabel : label;
}

constexpr size_t BLE_TRACKER_RING_SIZE = 32;

// Small mutex-guarded ring buffer fed by TrackerScanCallbacks::onResult() (running on the
// NimBLE host task) and drained by bleTrackerRun()'s UI loop (running on the main task).
// Follows the same xSemaphoreCreateMutex() pattern already used for shared scan-callback
// state in BLE_Suite.cpp's ScannerData.
struct BleTrackerHistory {
    BleTrackerSample samples[BLE_TRACKER_RING_SIZE];
    size_t head = 0; // next write index
    size_t count = 0;
    SemaphoreHandle_t mutex = nullptr;
    String targetMac;
    volatile bool active = false;

    void begin(const String &mac) {
        if (!mutex) mutex = xSemaphoreCreateMutex();
        if (!mutex || !xSemaphoreTake(mutex, portMAX_DELAY)) return;
        targetMac = mac;
        head = 0;
        count = 0;
        active = true;
        xSemaphoreGive(mutex);
    }

    // Stops accepting new samples from the callback; keeps the buffered history around in
    // case a future "review last session" view wants it.
    void end() {
        if (!mutex || !xSemaphoreTake(mutex, portMAX_DELAY)) return;
        active = false;
        xSemaphoreGive(mutex);
    }

    void append(int8_t rssi, uint16_t headingBucket, BleTrackerSampleSource source) {
        if (!mutex) return;
        // Called from the NimBLE host task: never block it - drop the sample if the main
        // loop happens to be reading the buffer at the same instant.
        if (!xSemaphoreTake(mutex, 0)) return;
        samples[head] = {rssi, headingBucket, millis(), source};
        head = (head + 1) % BLE_TRACKER_RING_SIZE;
        if (count < BLE_TRACKER_RING_SIZE) count++;
        xSemaphoreGive(mutex);
    }

    // Copies out the most recent sample (if any) in one locked pass, so the UI loop never
    // sees a torn/half-written entry.
    bool latest(BleTrackerSample &out) {
        if (!mutex || !xSemaphoreTake(mutex, 20 / portTICK_PERIOD_MS)) return false;
        bool has = count > 0;
        if (has) {
            size_t lastIdx = (head + BLE_TRACKER_RING_SIZE - 1) % BLE_TRACKER_RING_SIZE;
            out = samples[lastIdx];
        }
        xSemaphoreGive(mutex);
        return has;
    }
};

BleTrackerHistory g_history;

// Current heading bucket (16 x 22.5deg sectors), refreshed once per UI frame from
// imu_get_heading_delta_deg() - never read from the scan callback itself, since that runs on
// the NimBLE host task and must not touch the I2C bus. On boards without an IMU this simply
// stays 0 forever, which is exactly what every sample should be tagged with in that case.
volatile uint16_t g_currentHeadingBucket = 0;

constexpr int HEADING_BUCKETS = 16;
constexpr float HEADING_BUCKET_DEG = 360.0f / HEADING_BUCKETS;

uint16_t headingDegToBucket(float deg) {
    int bucket = ((int)((deg + HEADING_BUCKET_DEG / 2.0f) / HEADING_BUCKET_DEG)) % HEADING_BUCKETS;
    if (bucket < 0) bucket += HEADING_BUCKETS;
    return (uint16_t)bucket;
}

// Persistent onResult() callback (as opposed to ble_scan()'s one-shot getResults()): stays
// installed for the whole tracking session so every advertisement from the locked MAC,
// not just the ones caught in a short polling slice, becomes a sample.
class TrackerScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *device) override {
        if (!device || !g_history.active) return;
        String mac = device->getAddress().toString().c_str();
        if (!mac.equalsIgnoreCase(g_history.targetMac)) return;
        g_history.append(device->getRSSI(), g_currentHeadingBucket, TRACK_SRC_ESP32);
    }
};

TrackerScanCallbacks g_trackerScanCallbacks;

// Maps a raw RSSI (dBm) reading to a 0-100 "close/far" percentage using one shared default
// curve (per the plan's non-goal of per-hardware-revision calibration). -40dBm and closer is
// treated as 100% (touching distance); -100dBm and further is 0% (out of useful range).
int rssiToRangePercent(float rssi) {
    constexpr float RSSI_NEAR = -40.0f;
    constexpr float RSSI_FAR = -100.0f;
    float pct = (rssi - RSSI_FAR) / (RSSI_NEAR - RSSI_FAR) * 100.0f;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (int)(pct + 0.5f);
}

// Heuristic sweep-and-compare direction estimator: remembers the strongest RSSI seen per
// heading bucket, decaying every bucket over time so the arrow keeps following the target as
// the device (and the world around it) moves, rather than freezing on a one-off strong
// reading from minutes ago.
constexpr float HEADING_BEST_DECAY_DB_PER_SEC = 1.0f;
constexpr float HEADING_BEST_FLOOR = -127.0f;

struct BestHeadingTable {
    float bestRssi[HEADING_BUCKETS];
    unsigned long lastDecayMs = 0;
    int lastKnownBestBucket = -1;

    void reset() {
        for (int i = 0; i < HEADING_BUCKETS; i++) bestRssi[i] = HEADING_BEST_FLOOR;
        lastDecayMs = millis();
        lastKnownBestBucket = -1;
    }

    void decay() {
        unsigned long now = millis();
        float dtSec = (now - lastDecayMs) / 1000.0f;
        lastDecayMs = now;
        if (dtSec <= 0) return;
        float drop = HEADING_BEST_DECAY_DB_PER_SEC * dtSec;
        for (int i = 0; i < HEADING_BUCKETS; i++) {
            if (bestRssi[i] > HEADING_BEST_FLOOR) bestRssi[i] -= drop;
        }
    }

    void feed(uint16_t bucket, int8_t rssi) {
        if (bucket >= HEADING_BUCKETS) return;
        if (rssi > bestRssi[bucket]) bestRssi[bucket] = rssi;
        if (bestRssi[bucket] > HEADING_BEST_FLOOR) lastKnownBestBucket = bucket;
    }

    // Bucket with the strongest decayed RSSI, or lastKnownBestBucket if still valid, or -1.
    int bestBucket() const {
        int best = -1;
        float bestVal = HEADING_BEST_FLOOR;
        for (int i = 0; i < HEADING_BUCKETS; i++) {
            if (bestRssi[i] > bestVal) {
                bestVal = bestRssi[i];
                best = i;
            }
        }
        if (best < 0) return lastKnownBestBucket;
        return best;
    }
};

// Draws an arrow centered at (cx, cy) with the given radius, pointing `angleDeg` clockwise
// from straight up (0deg = up, matching a compass rose drawn on screen).
void drawHeadingArrow(int cx, int cy, int radius, float angleDeg, uint16_t color) {
    float rad = angleDeg * (PI / 180.0f);
    float dx = sinf(rad);
    float dy = -cosf(rad);

    int tipX = cx + (int)(dx * radius);
    int tipY = cy + (int)(dy * radius);

    // Two back corners of the arrowhead, swept +/-135deg from the tip direction.
    float backSpread = 135.0f * (PI / 180.0f);
    int backLen = radius / 2;
    int leftX = cx + (int)(sinf(rad + backSpread) * backLen);
    int leftY = cy + (int)(-cosf(rad + backSpread) * backLen);
    int rightX = cx + (int)(sinf(rad - backSpread) * backLen);
    int rightY = cy + (int)(-cosf(rad - backSpread) * backLen);

    tft.fillTriangle(tipX, tipY, leftX, leftY, rightX, rightY, color);
}

// Maps a 0.0 - 1.0 fraction to a smooth gradient color: Red (low/0.0) -> Yellow (mid/0.5) -> Green (high/1.0).
uint16_t getVuColor(float f) {
    f = constrain(f, 0.0f, 1.0f);
    uint8_t r = 0, g = 0, b = 0;
    if (f < 0.5f) {
        r = 255;
        g = (uint8_t)(255.0f * (f / 0.5f));
        b = 0;
    } else {
        r = (uint8_t)(255.0f * (1.0f - (f - 0.5f) / 0.5f));
        g = 255;
        b = 0;
    }
    return tft.color565(r, g, b);
}

// Produces a dimmed version of a 16-bit RGB565 color for unlit audio-mixer meter segments.
uint16_t getDimColor(uint16_t color) {
    uint8_t r = ((color >> 11) & 0x1F) / 4;
    uint8_t g = ((color >> 5) & 0x3F) / 4;
    uint8_t b = (color & 0x1F) / 4;
    return (uint16_t)((r << 11) | (g << 5) | b);
}

} // namespace

void bleTrackerLockTarget(const String &label, const String &mac) {
    options = {
        {"Track now", [=]() { bleTrackerRun(mac, label); }},
        {"Save as favorite & track",
         [=]() {
             String saved = promptForLabel(label);
             bruceConfig.addBleTrackerFavorite(saved, mac);
             bleTrackerRun(mac, saved);
         }},
    };
    addOptionToMainMenu();
    loopOptions(options);
    options.clear();
}

void bleTrackerScanAndPick() { bleTrackerPickFromScan(); }

void BleTrackerMenu() {
    options = {};

    for (const auto &fav : bruceConfig.bleTrackerFavorites) {
        String label = fav.label;
        String mac = fav.mac;
        options.emplace_back(label.c_str(), [=]() { bleTrackerLockTarget(label, mac); });
    }

    options.emplace_back("Live Scan", bleTrackerScanAndPick);

    addOptionToMainMenu();
    loopOptions(options, MENU_TYPE_SUBMENU, "BLE Tracker", 0, false);
    options.clear();
}

void bleTrackerRun(const String &targetMac, const String &label) {
    // Full-screen tracking view, sized for 240x135 (Cardputer/ADV) and smaller displays: a
    // title row, an audio-mixer style VU meter with smoothed RSSI & decaying peak hold, and a
    // dBm/stale readout. A persistent onResult() callback (TrackerScanCallbacks) keeps feeding
    // the ring buffer in the background while this loop just reads its latest sample and
    // redraws every ~50ms - there's no blocking delay in here longer than that, so EscPress
    // stays responsive.
    bool bleWasActiveBefore = BLEConnected || (BLEDevice::getServer() != nullptr);
#if !defined(LITE_VERSION)
    bleWasActiveBefore =
        bleWasActiveBefore || BLEStateManager::isBLEActive() || BLEStateManager::getActiveClientCount() > 0;
#endif

    if (!ble_scan_setup() || pBLEScan == nullptr) {
        displayError("Failed to init BLE scan");
        return;
    }
    pBLEScan->setActiveScan(false);                          // passive: never send scan requests
    pBLEScan->setScanCallbacks(&g_trackerScanCallbacks, true); // wantDuplicates=true: keep seeing the same MAC

    g_history.begin(targetMac);

    // IMU boards additionally show a direction arrow, built from a user-guided rotation
    // sweep - reset both the heading accumulator and the best-heading table so a previous
    // session/target's data never bleeds into this one.
    bool hasImu = imu_available();
    if (hasImu) {
        imu_init();
        imu_reset_heading();
        g_currentHeadingBucket = 0;
    }
    BestHeadingTable bestHeading;
    if (hasImu) bestHeading.reset();

    drawMainBorder();
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.drawCentreString(label, tftWidth / 2, BORDER_PAD_Y, SMOOTH_FONT);
    if (hasImu) printFootnote("Rotate slowly to locate");

    // Audio-mixer style VU meter geometry
    int barX = BORDER_PAD_X;
    int barY = BORDER_PAD_Y + FM * LH + 4;
    int barW = tftWidth - 2 * BORDER_PAD_X;
    int barH = 14;
    int readoutY = barY + barH + 4;

    constexpr int segW = 4;
    constexpr int segGap = 2;
    int numSegments = max(1, (barW - 4) / (segW + segGap));
    int actualBarW = numSegments * (segW + segGap) - segGap + 4;
    int actualBarX = barX + (barW - actualBarW) / 2;

    // Arrow area, only used on IMU boards: a small compass rose in the remaining vertical
    // space below the readout row, clamped so it never grows large enough to collide with
    // the sweep-prompt footnote on the smallest supported displays.
    int arrowCenterX = tftWidth / 2;
    int arrowCenterY = readoutY + LH * FP + 6 + 22;
    int arrowRadius = 20;
    if (arrowCenterY + arrowRadius + LH * FP + 4 > tftHeight) {
        arrowRadius = max(10, tftHeight - (readoutY + LH * FP + 6) - LH * FP - 8);
        arrowCenterY = readoutY + LH * FP + 6 + arrowRadius;
    }

    float emaRssi = -100.0f;
    bool emaInitialized = false;
    unsigned long lastSeenMs = 0;
    uint32_t lastProcessedSampleTs = 0;

    float peakFraction = 0.0f;
    unsigned long peakHoldUntilMs = 0;
    unsigned long lastFrameMs = millis();

    int lastDrawnSmoothedSeg = -1;
    int lastDrawnPeakSeg = -1;
    bool lastDrawnStale = false;
    float currentHeadingDeg = 0.0f;
    int lastDrawnAngleDeg = -999;
    int lastDrawnArrowBucket = -2;
    bool lastDrawnArrowStale = false;
    bool firstDraw = true;

    pBLEScan->start(0, false); // duration=0: scan indefinitely until stop()

    while (!check(EscPress)) {
        unsigned long nowMs = millis();
        float dtSec = (nowMs - lastFrameMs) / 1000.0f;
        if (dtSec < 0.0f || dtSec > 1.0f) dtSec = 0.05f;
        lastFrameMs = nowMs;

        if (hasImu) {
            currentHeadingDeg = imu_get_heading_delta_deg();
            g_currentHeadingBucket = headingDegToBucket(currentHeadingDeg);
            bestHeading.decay();
        }

        BleTrackerSample sample;
        if (g_history.latest(sample) && sample.timestamp != lastProcessedSampleTs) {
            lastProcessedSampleTs = sample.timestamp;
            constexpr float EMA_ALPHA = 0.25f;
            emaRssi = emaInitialized ? (EMA_ALPHA * sample.rssi + (1.0f - EMA_ALPHA) * emaRssi) : (float)sample.rssi;
            emaInitialized = true;
            lastSeenMs = sample.timestamp;

            float rawFraction = constrain(((float)sample.rssi - (-100.0f)) / 60.0f, 0.0f, 1.0f);
            if (rawFraction >= peakFraction) {
                peakFraction = rawFraction;
                peakHoldUntilMs = nowMs + 800; // Hold peak for 800ms
            }
            if (hasImu) bestHeading.feed(sample.headingBucket, sample.rssi);
        }

        bool stale = !emaInitialized || (nowMs - lastSeenMs) > 5000;

        // Peak decay after hold period expires (or faster decay when stale)
        if (stale) {
            peakFraction = max(0.0f, peakFraction - 1.5f * dtSec);
        } else if (nowMs > peakHoldUntilMs) {
            peakFraction = max(0.0f, peakFraction - 0.4f * dtSec);
        }

        float smoothedFraction =
            (!stale && emaInitialized) ? constrain((emaRssi - (-100.0f)) / 60.0f, 0.0f, 1.0f) : 0.0f;
        if (smoothedFraction > peakFraction) peakFraction = smoothedFraction;

        int smoothedSeg = (int)(smoothedFraction * numSegments + 0.01f);
        int peakSeg = (int)(peakFraction * numSegments + 0.01f);
        if (peakSeg >= numSegments) peakSeg = numSegments - 1;

        if (firstDraw || smoothedSeg != lastDrawnSmoothedSeg || peakSeg != lastDrawnPeakSeg ||
            stale != lastDrawnStale) {
            if (firstDraw) {
                tft.fillRect(actualBarX, barY, actualBarW, barH, bruceConfig.bgColor);
                tft.drawRect(actualBarX, barY, actualBarW, barH, TFT_DARKGREY);
            }

            for (int i = 0; i < numSegments; i++) {
                int segX = actualBarX + 2 + i * (segW + segGap);
                float f = (numSegments > 1) ? ((float)i / (float)(numSegments - 1)) : 0.0f;
                uint16_t color = getVuColor(f);

                bool isLit = (!stale && i < smoothedSeg);
                bool isPeak = (!stale && i == peakSeg && peakFraction > 0.02f);

                if (isLit) {
                    tft.fillRect(segX, barY + 2, segW, barH - 4, color);
                } else if (isPeak) {
                    tft.fillRect(segX, barY + 2, segW, barH - 4, TFT_WHITE);
                } else {
                    tft.fillRect(segX, barY + 2, segW, barH - 4, getDimColor(color));
                }
            }

            lastDrawnSmoothedSeg = smoothedSeg;
            lastDrawnPeakSeg = peakSeg;
            lastDrawnStale = stale;

            tft.fillRect(BORDER_PAD_X, readoutY, tftWidth - 2 * BORDER_PAD_X, LH * FP + 4, bruceConfig.bgColor);
            tft.setCursor(BORDER_PAD_X, readoutY);
            if (stale) {
                tft.setTextColor(TFT_DARKGREY, bruceConfig.bgColor);
                tft.print("stale, no adv. seen");
                tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            } else {
                int pct = (int)(smoothedFraction * 100.0f + 0.5f);
                int peakDbm = (int)(-100.0f + peakFraction * 60.0f + 0.5f);
                tft.printf("%d%% (%d dBm)  Peak: %d dBm", pct, (int)(emaRssi + 0.5f), peakDbm);
            }
        }

        if (hasImu) {
            int bestBucket = bestHeading.bestBucket();
            int angleDeg = 0;
            if (bestBucket >= 0) {
                float targetDeg = bestBucket * HEADING_BUCKET_DEG;
                angleDeg = (int)fmodf(targetDeg - currentHeadingDeg + 360.0f, 360.0f);
            }

            // Redraw if relative angle rotated by at least 2 degrees, or best bucket changed,
            // or staleness changed, or first frame.
            if (firstDraw || abs(angleDeg - lastDrawnAngleDeg) >= 2 || bestBucket != lastDrawnArrowBucket ||
                stale != lastDrawnArrowStale) {
                tft.fillCircle(arrowCenterX, arrowCenterY, arrowRadius + 2, bruceConfig.bgColor);
                tft.drawCircle(arrowCenterX, arrowCenterY, arrowRadius, bruceConfig.priColor);
                if (bestBucket >= 0) {
                    uint16_t arrowColor = stale ? TFT_DARKGREY : bruceConfig.priColor;
                    drawHeadingArrow(arrowCenterX, arrowCenterY, arrowRadius - 4, (float)angleDeg, arrowColor);
                } else {
                    // No packets seen yet - draw a center crosshair so the compass area is never empty/blank
                    tft.drawPixel(arrowCenterX, arrowCenterY, bruceConfig.priColor);
                    tft.drawPixel(arrowCenterX - 1, arrowCenterY, bruceConfig.priColor);
                    tft.drawPixel(arrowCenterX + 1, arrowCenterY, bruceConfig.priColor);
                    tft.drawPixel(arrowCenterX, arrowCenterY - 1, bruceConfig.priColor);
                    tft.drawPixel(arrowCenterX, arrowCenterY + 1, bruceConfig.priColor);
                }
                lastDrawnAngleDeg = angleDeg;
                lastDrawnArrowBucket = bestBucket;
                lastDrawnArrowStale = stale;
            }
        }

        firstDraw = false;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    g_history.end();
    if (pBLEScan) pBLEScan->stop();

    if (!bleWasActiveBefore) {
#if !defined(LITE_VERSION)
        if (!BLEStateManager::isBLEActive()) stopBLEStack();
#else
        stopBLEStack();
#endif
    }
}
