#include "ble_tracker.h"
#include "core/bus_HAL.h"
#include "core/config.h"
#include "core/display.h"
#include "core/imu.h"
#include "core/mykeyboard.h"
#include "core/utils.h"
#include "modules/ble/ble_common.h"
#include <NimBLEClient.h>
#include "core/radio_mem.h"
#if !defined(LITE_VERSION)
#include "BLE_Suite.h"
#include "modules/ble/ble_oui.h"
#include "modules/ble/gatt_explorer.h"
#endif
#include <math.h>

namespace {

struct BleTrackerDiscoveredDevice {
    uint8_t macBytes[6];
    char macStr[18];
    char name[32];
    char vendor[24];
    int8_t rssi;
    uint8_t addrType; // 0 = Public, 1 = Random
    uint32_t lastSeenMs;
    uint16_t packetCount;
};

constexpr size_t BLE_TRACKER_MAX_SCAN_DEVICES = 40;

struct BleTrackerScannerState {
    BleTrackerDiscoveredDevice devices[BLE_TRACKER_MAX_SCAN_DEVICES];
    size_t count = 0;
    uint32_t totalPackets = 0;
    SemaphoreHandle_t mutex = nullptr;
    volatile bool active = false;

    void reset() {
        if (!mutex) mutex = xSemaphoreCreateMutex();
        if (mutex && xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
            count = 0;
            totalPackets = 0;
            active = true;
            xSemaphoreGive(mutex);
        }
    }

    void stop() {
        if (!mutex) return;
        if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
            active = false;
            xSemaphoreGive(mutex);
        }
    }
};

static BleTrackerScannerState g_trackerScanState;

class BleTrackerLiveScanCallbacks : public NimBLEScanCallbacks {
public:
    void onDiscovered(const NimBLEAdvertisedDevice *dev) override {}

    void onResult(const NimBLEAdvertisedDevice *dev) override {
        if (!dev || !g_trackerScanState.active) return;
        if (!g_trackerScanState.mutex) return;

        // Try-take mutex with 0 timeout so NimBLE host task is never delayed
        if (xSemaphoreTake(g_trackerScanState.mutex, 0) != pdTRUE) return;

        g_trackerScanState.totalPackets++;

        const uint8_t *devVal = dev->getAddress().getVal();
        int8_t rssi = dev->getRSSI();
        uint32_t now = millis();

        // Check if device already exists in list (fast 6-byte binary comparison)
        for (size_t i = 0; i < g_trackerScanState.count; i++) {
            if (memcmp(g_trackerScanState.devices[i].macBytes, devVal, 6) == 0) {
                g_trackerScanState.devices[i].rssi = rssi;
                g_trackerScanState.devices[i].lastSeenMs = now;
                g_trackerScanState.devices[i].packetCount++;

                if (g_trackerScanState.devices[i].name[0] == '\0' && dev->haveName()) {
                    std::string n = dev->getName();
                    if (!n.empty() && n.length() < sizeof(g_trackerScanState.devices[i].name)) {
                        strncpy(g_trackerScanState.devices[i].name, n.c_str(), sizeof(g_trackerScanState.devices[i].name) - 1);
                        g_trackerScanState.devices[i].name[sizeof(g_trackerScanState.devices[i].name) - 1] = '\0';
                    }
                }
                if (g_trackerScanState.devices[i].vendor[0] == '\0') {
#if !defined(LITE_VERSION)
                    if (dev->haveManufacturerData()) {
                        std::string mfg = dev->getManufacturerData();
                        if (mfg.length() >= 2) {
                            uint16_t companyId = (uint8_t)mfg[0] | ((uint16_t)(uint8_t)mfg[1] << 8);
                            const char *comp = getBleCompanyIdName(companyId);
                            if (comp) {
                                strncpy(g_trackerScanState.devices[i].vendor, comp, sizeof(g_trackerScanState.devices[i].vendor) - 1);
                                g_trackerScanState.devices[i].vendor[sizeof(g_trackerScanState.devices[i].vendor) - 1] = '\0';
                            }
                        }
                    }
                    if (g_trackerScanState.devices[i].vendor[0] == '\0' && dev->getAddressType() == BLE_ADDR_PUBLIC) {
                        const char *oui = getBleOuiNameFromMacBytes(devVal);
                        if (oui) {
                            strncpy(g_trackerScanState.devices[i].vendor, oui, sizeof(g_trackerScanState.devices[i].vendor) - 1);
                            g_trackerScanState.devices[i].vendor[sizeof(g_trackerScanState.devices[i].vendor) - 1] = '\0';
                        }
                    }
#endif
                }
                xSemaphoreGive(g_trackerScanState.mutex);
                return;
            }
        }

        // New device: append if space available
        if (g_trackerScanState.count < BLE_TRACKER_MAX_SCAN_DEVICES) {
            auto &d = g_trackerScanState.devices[g_trackerScanState.count];
            memcpy(d.macBytes, devVal, 6);
            snprintf(d.macStr, sizeof(d.macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                     devVal[5], devVal[4], devVal[3], devVal[2], devVal[1], devVal[0]);

            if (dev->haveName()) {
                std::string n = dev->getName();
                if (!n.empty()) {
                    strncpy(d.name, n.c_str(), sizeof(d.name) - 1);
                    d.name[sizeof(d.name) - 1] = '\0';
                } else {
                    d.name[0] = '\0';
                }
            } else {
                d.name[0] = '\0';
            }

            d.vendor[0] = '\0';
#if !defined(LITE_VERSION)
            if (dev->haveManufacturerData()) {
                std::string mfg = dev->getManufacturerData();
                if (mfg.length() >= 2) {
                    uint16_t companyId = (uint8_t)mfg[0] | ((uint16_t)(uint8_t)mfg[1] << 8);
                    const char *comp = getBleCompanyIdName(companyId);
                    if (comp) {
                        strncpy(d.vendor, comp, sizeof(d.vendor) - 1);
                        d.vendor[sizeof(d.vendor) - 1] = '\0';
                    }
                }
            }
            if (d.vendor[0] == '\0' && dev->getAddressType() == BLE_ADDR_PUBLIC) {
                const char *oui = getBleOuiNameFromMacBytes(devVal);
                if (oui) {
                    strncpy(d.vendor, oui, sizeof(d.vendor) - 1);
                    d.vendor[sizeof(d.vendor) - 1] = '\0';
                }
            }
#endif

            d.rssi = rssi;
            d.addrType = dev->getAddressType();
            d.lastSeenMs = now;
            d.packetCount = 1;
            g_trackerScanState.count++;
        }

        xSemaphoreGive(g_trackerScanState.mutex);
    }
};

static BleTrackerLiveScanCallbacks g_trackerLiveScanCallbacks;

// Dedicated, thread-safe Live Scanner for BLE Tracker.
// Uses passive continuous scanning with zero heap allocations on the NimBLE host task,
// capturing all nearby trackers, beacons (AirTags, Tile, SmartTags), and standard BLE devices.
void bleTrackerPickFromScan() {
    // 1. Drain residual keys from menu selection
    vTaskDelay(pdMS_TO_TICKS(150));
    check(SelPress);
    check(EscPress);
    _getKeyPress();

    // 2. Check RAM availability
    if (!radioHasMemForBle()) {
        displayError("Low RAM: free WiFi/SD first", true);
        return;
    }

    bool bleWasActiveBefore = BLEConnected || (BLEDevice::getServer() != nullptr);
#if !defined(LITE_VERSION)
    bleWasActiveBefore =
        bleWasActiveBefore || BLEStateManager::isBLEActive() || BLEStateManager::getActiveClientCount() > 0;
#endif

    // 3. Setup BLE scan
    if (!ble_scan_setup() || pBLEScan == nullptr) {
        displayError("Failed to init BLE scan");
        return;
    }

    g_trackerScanState.reset();

    pBLEScan->setScanCallbacks(&g_trackerLiveScanCallbacks, true);
    pBLEScan->setActiveScan(false); // passive: captures all trackers/beacons without transmitting
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    pBLEScan->setDuplicateFilter(false);
    pBLEScan->setMaxResults(0);
    pBLEScan->clearResults();

    drawMainBorder(true);

    pBLEScan->start(0, false);

    int selectedIdx = 0;
    int scrollOffset = 0;
    uint32_t lastUiUpdate = 0;
    int animFrame = 0;
    const char *spinner = "|/-\\";
    bool needsRedraw = true;
    bool scanPaused = false;
    String pickedName = "";
    String pickedMac = "";
    bool devicePicked = false;

    // Snapshot buffer for UI rendering to minimize lock hold time (static to protect task stack)
    static BleTrackerDiscoveredDevice uiDevices[BLE_TRACKER_MAX_SCAN_DEVICES];
    size_t uiCount = 0;
    uint32_t uiPackets = 0;

    int lineH = 8 * FP + 4;
    int headerY = BORDER_PAD_Y;
    int listStartY = headerY + lineH + 4;
    int footerY = tftHeight - BORDER_PAD_Y - lineH;
    int visibleRows = (footerY - listStartY) / lineH;
    if (visibleRows < 1) visibleRows = 1;

    while (true) {
        // Handle physical buttons & encoder
        bool up = check(PrevPress) || check(UpPress);
        bool down = check(NextPress) || check(DownPress);
        bool sel = check(SelPress);
        bool esc = check(EscPress);

#if defined(HAS_ENCODER)
        int encSteps = getEncoderSteps();
        if (encSteps > 0) down = true;
        else if (encSteps < 0) up = true;
#endif

        keyStroke k = _getKeyPress();
        if (k.pressed || !k.word.empty()) {
            if (k.del || k.exit_key) {
                esc = true;
            }
            if (k.enter) {
                sel = true;
            }
            for (auto ch : k.word) {
                char lowerKey = tolower(ch);
                if (lowerKey == '`' || lowerKey == 'q' || lowerKey == 0x1B) {
                    esc = true;
                } else if (lowerKey == 's') {
                    sel = true;
                } else if (lowerKey == 'p' || lowerKey == ' ') {
                    scanPaused = !scanPaused;
                    if (scanPaused) {
                        pBLEScan->stop();
                    } else {
                        pBLEScan->start(0, false);
                    }
                    needsRedraw = true;
                } else if (lowerKey == 'c') {
                    g_trackerScanState.reset();
                    selectedIdx = 0;
                    scrollOffset = 0;
                    needsRedraw = true;
                }
            }
        }

        if (esc) {
            break;
        }

        // Copy snapshot from scanner state under lock
        if (millis() - lastUiUpdate > 100 || needsRedraw) {
            if (g_trackerScanState.mutex && xSemaphoreTake(g_trackerScanState.mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                uiCount = g_trackerScanState.count;
                uiPackets = g_trackerScanState.totalPackets;
                memcpy(uiDevices, g_trackerScanState.devices, uiCount * sizeof(BleTrackerDiscoveredDevice));
                xSemaphoreGive(g_trackerScanState.mutex);
            }
            needsRedraw = true;
        }

        if (up) {
            if (selectedIdx > 0) {
                selectedIdx--;
                needsRedraw = true;
            }
        }
        if (down) {
            if (selectedIdx + 1 < (int)uiCount) {
                selectedIdx++;
                needsRedraw = true;
            }
        }

        if (sel && uiCount > 0 && selectedIdx >= 0 && selectedIdx < (int)uiCount) {
            pickedMac = String(uiDevices[selectedIdx].macStr);
            if (uiDevices[selectedIdx].name[0] != '\0') {
                pickedName = String(uiDevices[selectedIdx].name);
            } else if (uiDevices[selectedIdx].vendor[0] != '\0') {
                pickedName = String(uiDevices[selectedIdx].vendor) + " (" + pickedMac.substring(9) + ")";
            } else {
                pickedName = pickedMac;
            }
            devicePicked = true;
            break;
        }

        // Keep scroll offset aligned with selected index
        if (selectedIdx < scrollOffset) {
            scrollOffset = selectedIdx;
        }
        if (selectedIdx >= scrollOffset + visibleRows) {
            scrollOffset = selectedIdx - visibleRows + 1;
        }

        // Render UI
        uint32_t now = millis();
        if (needsRedraw || (now - lastUiUpdate >= 120)) {
            lastUiUpdate = now;
            needsRedraw = false;
            animFrame = (animFrame + 1) % 4;

            // 1. Header (Title & Status)
            tft.setTextSize(FP);
            tft.fillRect(BORDER_PAD_X, headerY, tftWidth - 2 * BORDER_PAD_X, lineH, bruceConfig.bgColor);
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            String title = "BLE LIVE SCAN ";
            title += scanPaused ? "[PAUSED]" : ("[" + String(spinner[animFrame]) + "]");
            tft.drawString(title, BORDER_PAD_X, headerY);

            String countStr = "Dev: " + String(uiCount) + " Pkt: " + String(uiPackets);
            tft.drawRightString(countStr, tftWidth - BORDER_PAD_X, headerY, 1);

            tft.drawFastHLine(BORDER_PAD_X, headerY + lineH - 1, tftWidth - 2 * BORDER_PAD_X, TFT_DARKGREY);

            // 2. Device List rows
            for (int r = 0; r < visibleRows; r++) {
                int itemIdx = scrollOffset + r;
                int rowY = listStartY + r * lineH;

                if (itemIdx < (int)uiCount) {
                    bool isSel = (itemIdx == selectedIdx);
                    uint16_t bg = isSel ? bruceConfig.priColor : bruceConfig.bgColor;
                    uint16_t fg = isSel ? bruceConfig.bgColor : bruceConfig.priColor;

                    tft.fillRect(BORDER_PAD_X, rowY, tftWidth - 2 * BORDER_PAD_X, lineH, bg);
                    tft.setTextColor(fg, bg);

                    // Draw RSSI signal indicator
                    int8_t rssi = uiDevices[itemIdx].rssi;
                    int rssiX = BORDER_PAD_X + 2;
                    int bars = (rssi > -60) ? 4 : ((rssi > -75) ? 3 : ((rssi > -88) ? 2 : 1));
                    for (int b = 0; b < 4; b++) {
                        int h = 2 + b * 2;
                        if (b < bars) {
                            tft.fillRect(rssiX + b * 3, rowY + lineH - 3 - h, 2, h, fg);
                        } else {
                            tft.drawFastHLine(rssiX + b * 3, rowY + lineH - 4, 2, fg);
                        }
                    }

                    int textX = rssiX + 16;
                    int maxTextW = tftWidth - textX - BORDER_PAD_X - 4;

                    String devLabel = "";
                    if (uiDevices[itemIdx].name[0] != '\0') {
                        devLabel = String(uiDevices[itemIdx].name);
                    } else if (uiDevices[itemIdx].vendor[0] != '\0') {
                        devLabel = "[" + String(uiDevices[itemIdx].vendor) + "] " + String(uiDevices[itemIdx].macStr);
                    } else {
                        devLabel = String(uiDevices[itemIdx].macStr);
                    }

                    devLabel += " " + String(rssi) + "d";
                    if (uiDevices[itemIdx].addrType == BLE_ADDR_PUBLIC) {
                        devLabel += " [P]";
                    }

                    // Truncate to fit screen width
                    while (devLabel.length() > 3 && tft.textWidth(devLabel.c_str()) > maxTextW) {
                        devLabel.remove(devLabel.length() - 1);
                    }
                    tft.drawString(devLabel, textX, rowY + 1);
                } else {
                    tft.fillRect(BORDER_PAD_X, rowY, tftWidth - 2 * BORDER_PAD_X, lineH, bruceConfig.bgColor);
                    if (uiCount == 0 && r == 0) {
                        tft.setTextColor(bruceConfig.secColor, bruceConfig.bgColor);
                        tft.drawString("Listening for BLE beacons...", BORDER_PAD_X + 4, rowY + 1);
                    }
                }
            }

            // 3. Footer
            tft.fillRect(BORDER_PAD_X, footerY, tftWidth - 2 * BORDER_PAD_X, lineH, bruceConfig.bgColor);
            tft.setTextColor(getColorVariation(bruceConfig.priColor, 8, -1), bruceConfig.bgColor);
            tft.drawCentreString("ENTER/SEL lock   ESC back", tftWidth / 2, footerY + 1, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(25));
    }

    // Stop scanner cleanly
    g_trackerScanState.stop();
    if (pBLEScan) {
        pBLEScan->stop();
        pBLEScan->clearResults();
        pBLEScan->setScanCallbacks(nullptr);
    }

    if (!bleWasActiveBefore) {
        stopBLEStack();
    }

    // Drain keys
    vTaskDelay(pdMS_TO_TICKS(150));
    check(SelPress);
    check(EscPress);
    _getKeyPress();

    if (devicePicked && !pickedMac.isEmpty()) {
        bleTrackerLockTarget(pickedName, pickedMac);
    }
}

// Free-form label prompt reused by both the "save as favorite" flow below and (in a later
// revision) the favorites list itself.
String promptForLabel(const String &defaultLabel) {
    String label = keyboard(defaultLabel, 24, "Label for favorite");
    return label.isEmpty() ? defaultLabel : label;
}

constexpr size_t BLE_TRACKER_RING_SIZE = 64;

// Small mutex-guarded ring buffer fed by TrackerScanCallbacks::onResult() (running on the
// NimBLE host task) and drained by bleTrackerRun()'s UI loop (running on the main task).
// Follows the same xSemaphoreCreateMutex() pattern already used for shared scan-callback
// state in BLE_Suite.cpp's ScannerData.
struct BleTrackerHistory {
    BleTrackerSample samples[BLE_TRACKER_RING_SIZE];
    size_t head = 0; // next write index
    size_t count = 0;
    SemaphoreHandle_t mutex = nullptr;
    uint8_t targetMacBytes[6] = {0};
    volatile bool hasTarget = false;
    volatile bool active = false;

    void begin(const String &mac) {
        if (!mutex) mutex = xSemaphoreCreateMutex();
        if (!mutex || !xSemaphoreTake(mutex, portMAX_DELAY)) return;
        NimBLEAddress addr(std::string(mac.c_str()), 0);
        memcpy(targetMacBytes, addr.getVal(), 6);
        hasTarget = true;
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
        hasTarget = false;
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

// Current heading bucket (36 x 10deg sectors), refreshed once per UI frame from
// imu_get_heading_delta_deg() - never read from the scan callback itself, since that runs on
// the NimBLE host task and must not touch the I2C bus. On boards without an IMU this simply
// stays 0 forever, which is exactly what every sample should be tagged with in that case.
volatile uint16_t g_currentHeadingBucket = 0;

constexpr int HEADING_BUCKETS = 36;
constexpr float HEADING_BUCKET_DEG = 360.0f / HEADING_BUCKETS; // 10 degrees per bin

uint16_t headingDegToBucket(float deg) {
    float norm = fmodf(deg, 360.0f);
    if (norm < 0.0f) norm += 360.0f;
    int bucket = ((int)((norm + HEADING_BUCKET_DEG / 2.0f) / HEADING_BUCKET_DEG)) % HEADING_BUCKETS;
    if (bucket < 0) bucket += HEADING_BUCKETS;
    return (uint16_t)bucket;
}

// Persistent onResult() callback (as opposed to ble_scan()'s one-shot getResults()): stays
// installed for the whole tracking session so every advertisement from the locked MAC,
// not just the ones caught in a short polling slice, becomes a sample.
// Fast 6-byte binary comparison with zero dynamic memory allocation on the Bluetooth stack.
class TrackerScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *device) override {
        if (!device || !g_history.active || !g_history.hasTarget) return;
        if (memcmp(device->getAddress().getVal(), g_history.targetMacBytes, 6) != 0) return;
        g_history.append(device->getRSSI(), g_currentHeadingBucket, TRACK_SRC_ESP32);
    }
};

TrackerScanCallbacks g_trackerScanCallbacks;

// Direction estimator using Circular Vector Average (Angular Centroid):
// Accumulates EMA signal levels across 36 angular bins (10° resolution) and computes
// the 2D vector centroid sum to provide a smooth, continuous target direction robust
// against multi-path reflections and noise spikes.
constexpr float HEADING_BEST_DECAY_DB_PER_SEC = 2.0f;
constexpr float HEADING_NOISE_FLOOR = -100.0f;

enum HeadingConfidence {
    CONFIDENCE_LOW = 0,  // Red: Need more samples / sweep sectors
    CONFIDENCE_MED = 1,  // Yellow: Gathering sweep data, refining estimate
    CONFIDENCE_HIGH = 2  // Green: High multi-sector confidence
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

    // Circular Vector Average (Angular Centroid) across all heading bins with confidence scoring
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

                float sig = binRssi[i] - HEADING_NOISE_FLOOR; // 0..60
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

        // Confidence estimation: rewards both packet count, angular spread, and lobe distinctiveness
        if (totalSamples < 6 || populatedSectors < 3) {
            conf = CONFIDENCE_LOW;
        } else if (totalSamples < 14 || populatedSectors < 5 || directivity < 0.25f) {
            conf = CONFIDENCE_MED;
        } else {
            conf = CONFIDENCE_HIGH;
        }

        if (totalWeight > 8.0f && vectorMag > 4.0f) {
            float angleRad = atan2f(sumY, sumX);
            float targetDeg = angleRad * (180.0f / M_PI);
            targetDeg = fmodf(targetDeg + 360.0f, 360.0f);

            if (!hasTarget) {
                lastTargetDeg = targetDeg;
                hasTarget = true;
            } else {
                // Circular low-pass filter to smooth angle changes
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
    while (true) {
        bool isFav = bruceConfig.isBleTrackerFavorite(mac);
        std::vector<Option> targetOptions = {
            {"Track now", [=]() { bleTrackerRun(mac, label); }},
        };
        if (isFav) {
            targetOptions.push_back({"Remove from favorites", [=]() {
                bruceConfig.removeBleTrackerFavorite(mac);
                displaySuccess("Removed from favs", true);
            }});
        } else {
            targetOptions.push_back({"Save as favorite & track", [=]() {
                String saved = promptForLabel(label);
                bruceConfig.addBleTrackerFavorite(saved, mac);
                bleTrackerRun(mac, saved);
            }});
        }
        targetOptions.push_back({"< Back", []() {}});

        int chosen = loopOptions(targetOptions, MENU_TYPE_SUBMENU, "Target Options");
        if (chosen < 0 || chosen == (int)targetOptions.size() - 1) {
            break;
        }
    }
}

void bleTrackerScanAndPick() { bleTrackerPickFromScan(); }

void BleTrackerMenu() {
    while (true) {
        std::vector<Option> trackerOptions;

        for (const auto &fav : bruceConfig.bleTrackerFavorites) {
            String label = fav.label;
            String mac = fav.mac;
            trackerOptions.emplace_back(label, [=]() { bleTrackerLockTarget(label, mac); });
        }

        trackerOptions.emplace_back("Live Scan", bleTrackerScanAndPick);

        if (!bruceConfig.bleTrackerFavorites.empty()) {
            trackerOptions.emplace_back("Clear all favorites", [=]() {
                bruceConfig.clearBleTrackerFavorites();
                displaySuccess("Favorites cleared", true);
            });
        }

        trackerOptions.emplace_back("< Back to BLE Menu", []() {});

        int chosen = loopOptions(trackerOptions, MENU_TYPE_SUBMENU, "BLE Tracker", 0, false);
        if (chosen < 0 || chosen == (int)trackerOptions.size() - 1) {
            break;
        }
    }
}

void bleTrackerRun(const String &targetMac, const String &label, NimBLEClient *pClient) {
    // Full-screen tracking view, sized for 240x135 (Cardputer/ADV) and smaller displays: a
    // title row with [ACTIVE]/[PASSIVE] status indicator, an audio-mixer style VU meter with
    // smoothed RSSI & decaying peak hold, and a dBm/stale readout. A persistent onResult()
    // callback (TrackerScanCallbacks) keeps feeding the ring buffer in the background, while
    // active central connections poll link-layer RSSI directly.
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
    // sweep - calibrate resting gyro bias and reset both the heading accumulator and the
    // best-heading table so a previous session/target's data never bleeds into this one.
    bool hasImu = imu_available();
    if (hasImu) {
        drawMainBorder();
        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
        tft.drawCentreString("IMU Calibration", tftWidth / 2, BORDER_PAD_Y, SMOOTH_FONT);
        tft.drawCentreString("Hold device still...", tftWidth / 2, tftHeight / 2 - 16, SMOOTH_FONT);

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
        g_currentHeadingBucket = 0;
    }
    BestHeadingTable bestHeading;
    if (hasImu) bestHeading.reset();

    drawMainBorder();
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.drawCentreString(label, tftWidth / 2, BORDER_PAD_Y, SMOOTH_FONT);

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

    // Arrow and text geometry on IMU boards:
    // Placed on the left side (X ~ 28) with compass rose, and informative sweep / bearing
    // text on the right side (X ~ 60..235), avoiding any overlap on 240x135 displays.
    int arrowCenterX = BORDER_PAD_X + 24;
    int arrowCenterY = readoutY + LH * FP + 24;
    int arrowRadius = 18;
    if (arrowCenterY + arrowRadius + 4 > tftHeight) {
        arrowRadius = max(10, (tftHeight - 4 - (readoutY + LH * FP + 4)) / 2);
        arrowCenterY = (readoutY + LH * FP + 4) + arrowRadius + 2;
    }
    int infoTextX = arrowCenterX + arrowRadius + 12;

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
    bool lastDrawnResolved = false;
    bool lastDrawnArrowStale = false;
    HeadingConfidence lastDrawnConf = (HeadingConfidence)-1;
    int lastDrawnConnState = -1;
    bool firstDraw = true;

    pBLEScan->start(0, false); // duration=0: scan indefinitely until stop()

    while (!check(EscPress)) {
        unsigned long nowMs = millis();
        float dtSec = (nowMs - lastFrameMs) / 1000.0f;
        if (dtSec < 0.0f || dtSec > 1.0f) dtSec = 0.05f;
        lastFrameMs = nowMs;

        // Active connection RSSI polling (when tracking directly from GATT Explorer while connected)
        bool isActivelyConnected = (pClient != nullptr && pClient->isConnected());
        if (isActivelyConnected) {
            int connRssi = pClient->getRssi();
            if (connRssi != 0) {
                g_history.append((int8_t)connRssi, g_currentHeadingBucket, TRACK_SRC_ESP32);
            }
        }

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

        // Mode badge in top right header: [ACTIVE] vs [PASSIVE]
        int connState = isActivelyConnected ? 1 : 0;
        if (firstDraw || connState != lastDrawnConnState) {
            int badgeW = FP * LH * 5 + 4;
            tft.fillRect(tftWidth - BORDER_PAD_X - badgeW, BORDER_PAD_Y, badgeW, FM * LH, bruceConfig.bgColor);
            tft.setTextColor(isActivelyConnected ? TFT_GREEN : TFT_CYAN, bruceConfig.bgColor);
            tft.drawRightString(
                isActivelyConnected ? "[ACTIVE]" : "[PASSIVE]", tftWidth - BORDER_PAD_X, BORDER_PAD_Y, SMOOTH_FONT
            );
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            lastDrawnConnState = connState;
        }

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
                tft.print("stale, no signal seen");
                tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            } else {
                int pct = (int)(smoothedFraction * 100.0f + 0.5f);
                int peakDbm = (int)(-100.0f + peakFraction * 60.0f + 0.5f);
                tft.printf("%d%% (%d dBm)  Peak: %d dBm", pct, (int)(emaRssi + 0.5f), peakDbm);
            }
        }

        if (hasImu) {
            bool resolved = false;
            HeadingConfidence conf = CONFIDENCE_LOW;
            float targetDeg = bestHeading.getTargetBearingDeg(resolved, conf);
            int angleDeg = 0;
            if (resolved) {
                angleDeg = (int)fmodf(targetDeg - currentHeadingDeg + 360.0f, 360.0f);
            }

            // Redraw if relative angle rotated by at least 2 degrees, or resolution status changed,
            // or confidence changed, or staleness changed, or first frame.
            if (firstDraw || abs(angleDeg - lastDrawnAngleDeg) >= 2 || resolved != lastDrawnResolved ||
                conf != lastDrawnConf || stale != lastDrawnArrowStale) {
                tft.fillCircle(arrowCenterX, arrowCenterY, arrowRadius + 2, bruceConfig.bgColor);

                uint16_t bgCol, ringCol, arrowCol;
                if (stale) {
                    bgCol = bruceConfig.bgColor;
                    ringCol = TFT_DARKGREY;
                    arrowCol = TFT_DARKGREY;
                } else {
                    switch (conf) {
                        case CONFIDENCE_LOW:
                            bgCol = tft.color565(55, 12, 12);     // Dark red background
                            ringCol = tft.color565(220, 45, 45);  // Red ring
                            break;
                        case CONFIDENCE_MED:
                            bgCol = tft.color565(55, 45, 10);     // Dark amber background
                            ringCol = tft.color565(230, 180, 25); // Yellow/amber ring
                            break;
                        case CONFIDENCE_HIGH:
                        default:
                            bgCol = tft.color565(12, 55, 20);     // Dark green background
                            ringCol = tft.color565(45, 210, 75);  // Green ring
                            break;
                    }
                    arrowCol = TFT_WHITE;
                }

                tft.fillCircle(arrowCenterX, arrowCenterY, arrowRadius, bgCol);
                tft.drawCircle(arrowCenterX, arrowCenterY, arrowRadius, ringCol);

                if (resolved) {
                    drawHeadingArrow(arrowCenterX, arrowCenterY, arrowRadius - 3, (float)angleDeg, arrowCol);
                } else {
                    // No confident direction yet - draw a center crosshair so the compass area is never empty/blank
                    tft.drawPixel(arrowCenterX, arrowCenterY, arrowCol);
                    tft.drawPixel(arrowCenterX - 1, arrowCenterY, arrowCol);
                    tft.drawPixel(arrowCenterX + 1, arrowCenterY, arrowCol);
                    tft.drawPixel(arrowCenterX, arrowCenterY - 1, arrowCol);
                    tft.drawPixel(arrowCenterX, arrowCenterY + 1, arrowCol);
                }

                // Render side text cleanly separated from compass rose
                int textW = tftWidth - BORDER_PAD_X - infoTextX;
                if (textW > 20) {
                    int line1Y = arrowCenterY - arrowRadius + 2;
                    int line2Y = line1Y + LH * FP + 3;
                    tft.fillRect(infoTextX, line1Y, textW, LH * FP * 2 + 8, bruceConfig.bgColor);

                    tft.setTextSize(FP);
                    tft.setCursor(infoTextX, line1Y);
                    if (resolved) {
                        String dirStr;
                        if (angleDeg <= 25 || angleDeg >= 335) dirStr = "Ahead";
                        else if (angleDeg < 70) dirStr = "+" + String(angleDeg) + "d R";
                        else if (angleDeg <= 110) dirStr = "Right";
                        else if (angleDeg < 160) dirStr = "Behind-R";
                        else if (angleDeg <= 200) dirStr = "Behind";
                        else if (angleDeg < 250) dirStr = "Behind-L";
                        else if (angleDeg <= 290) dirStr = "Left";
                        else dirStr = "-" + String(360 - angleDeg) + "d L";

                        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
                        tft.print("Dir: ");
                        tft.print(dirStr);
                    } else {
                        tft.setTextColor(TFT_DARKGREY, bruceConfig.bgColor);
                        tft.print("Target: locating");
                    }

                    tft.setCursor(infoTextX, line2Y);
                    if (stale) {
                        tft.setTextColor(TFT_DARKGREY, bruceConfig.bgColor);
                        tft.print("Rotate to sweep");
                    } else if (conf == CONFIDENCE_LOW) {
                        tft.setTextColor(tft.color565(240, 80, 80), bruceConfig.bgColor);
                        tft.print("Sweep: need data");
                    } else if (conf == CONFIDENCE_MED) {
                        tft.setTextColor(tft.color565(240, 200, 50), bruceConfig.bgColor);
                        tft.print("Sweep: refining");
                    } else {
                        tft.setTextColor(tft.color565(60, 220, 80), bruceConfig.bgColor);
                        tft.print("Sweep: locked");
                    }
                    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
                }

                lastDrawnAngleDeg = angleDeg;
                lastDrawnResolved = resolved;
                lastDrawnConf = conf;
                lastDrawnArrowStale = stale;
            }
        }

        firstDraw = false;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    g_history.end();
    if (pBLEScan) {
        pBLEScan->stop();
        pBLEScan->clearResults();
        pBLEScan->setScanCallbacks(nullptr);
    }

    if (pClient == nullptr && !bleWasActiveBefore) {
#if !defined(LITE_VERSION)
        if (!BLEStateManager::isBLEActive()) stopBLEStack();
#else
        stopBLEStack();
#endif
    }
}
