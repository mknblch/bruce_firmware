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

#if defined(LITE_VERSION)
struct BleTrackerDiscoveredDevice {
    uint8_t macBytes[6];
    char macStr[18];
    char name[32];
    char vendor[24];
    int8_t rssi;
    uint8_t addrType; // 0 = Public, 1 = Random
    uint32_t lastSeenMs;
    uint32_t packetCount;
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
            count = 0;
            totalPackets = 0;
            xSemaphoreGive(mutex);
        }
    }
};

static BleTrackerScannerState g_trackerScanState;

class BleTrackerLiveScanCallbacks : public NimBLEScanCallbacks {
public:
    void onDiscovered(const NimBLEAdvertisedDevice *dev) override {}

    void onResult(const NimBLEAdvertisedDevice *dev) override {
        if (!dev) return;
        if (!g_trackerScanState.mutex) return;

        const uint8_t *devVal = dev->getAddress().getVal();
        const int8_t rssi = dev->getRSSI();
        const uint8_t addrType = dev->getAddressType();
        const uint32_t now = millis();
        char name[32] = {0};
        char vendor[24] = {0};

        // Resolve optional metadata before taking the shared-state mutex.
        if (dev->haveName()) {
            std::string n = dev->getName();
            if (!n.empty()) {
                strncpy(name, n.c_str(), sizeof(name) - 1);
                name[sizeof(name) - 1] = '\0';
            }
        }
#if !defined(LITE_VERSION)
        if (dev->haveManufacturerData()) {
            std::string mfg = dev->getManufacturerData();
            if (mfg.length() >= 2) {
                uint16_t companyId = (uint8_t)mfg[0] | ((uint16_t)(uint8_t)mfg[1] << 8);
                const char *comp = getBleCompanyIdName(companyId);
                if (comp) {
                    strncpy(vendor, comp, sizeof(vendor) - 1);
                    vendor[sizeof(vendor) - 1] = '\0';
                }
            }
        }
        if (vendor[0] == '\0' && addrType == BLE_ADDR_PUBLIC) {
            const char *oui = getBleOuiNameFromMacBytes(devVal);
            if (oui) {
                strncpy(vendor, oui, sizeof(vendor) - 1);
                vendor[sizeof(vendor) - 1] = '\0';
            }
        }
#endif

        // Try-take mutex with 0 timeout so NimBLE host task is never delayed
        if (xSemaphoreTake(g_trackerScanState.mutex, 0) != pdTRUE) return;
        if (!g_trackerScanState.active) {
            xSemaphoreGive(g_trackerScanState.mutex);
            return;
        }

        if (g_trackerScanState.totalPackets < UINT32_MAX) g_trackerScanState.totalPackets++;

        // Check if device already exists in list (fast 6-byte binary comparison)
        for (size_t i = 0; i < g_trackerScanState.count; i++) {
            if (memcmp(g_trackerScanState.devices[i].macBytes, devVal, 6) == 0 &&
                g_trackerScanState.devices[i].addrType == addrType) {
                g_trackerScanState.devices[i].rssi = rssi;
                g_trackerScanState.devices[i].lastSeenMs = now;
                if (g_trackerScanState.devices[i].packetCount < UINT32_MAX) {
                    g_trackerScanState.devices[i].packetCount++;
                }

                if (g_trackerScanState.devices[i].name[0] == '\0' && name[0] != '\0') {
                    memcpy(g_trackerScanState.devices[i].name, name, sizeof(name));
                }
                if (g_trackerScanState.devices[i].vendor[0] == '\0') {
                    memcpy(g_trackerScanState.devices[i].vendor, vendor, sizeof(vendor));
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

            memcpy(d.name, name, sizeof(name));

            memcpy(d.vendor, vendor, sizeof(vendor));

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

// Dedicated passive scanner used only when the tracker owns the BLE stack.
// NimBLE metadata accessors may allocate, so they are evaluated before taking the UI-state mutex.
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

    if (bleWasActiveBefore || pBLEScan != nullptr) {
        displayError("BLE scanner already in use");
        return;
    }

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

    if (!pBLEScan->start(0, false)) {
        g_trackerScanState.stop();
        pBLEScan->setScanCallbacks(nullptr);
        stopBLEStack();
        displayError("Failed to start BLE scan");
        return;
    }

    int selectedIdx = 0;
    int scrollOffset = 0;
    uint32_t lastUiUpdate = 0;
    int animFrame = 0;
    const char *spinner = "|/-\\";
    bool needsRedraw = true;
    bool scanPaused = false;
    String pickedName = "";
    String pickedMac = "";
    uint8_t pickedAddrType = 0xFF;
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
        bool up = false;
        bool down = false;
        bool sel = check(SelPress);
        bool esc = check(EscPress);

#if defined(HAS_ENCODER)
        int32_t encSteps = drainRotarySteps();
        if (encSteps != 0) {
            check(PrevPress);
            check(NextPress);
            check(UpPress);
            check(DownPress);
            while (encSteps > 0) {
                if (selectedIdx > 0) {
                    selectedIdx--;
                    needsRedraw = true;
                }
                encSteps--;
            }
            while (encSteps < 0) {
                if (selectedIdx + 1 < (int)uiCount) {
                    selectedIdx++;
                    needsRedraw = true;
                }
                encSteps++;
            }
            PrevPress = false;
            NextPress = false;
            UpPress = false;
            DownPress = false;
        } else
#endif
        {
            if (check(PrevPress) || check(UpPress)) up = true;
            if (check(NextPress) || check(DownPress)) down = true;
        }

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
                    if (!scanPaused) {
                        pBLEScan->stop();
                        scanPaused = true;
                    } else {
                        scanPaused = !pBLEScan->start(0, false);
                        if (scanPaused) displayError("Failed to resume BLE scan");
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
            pickedAddrType = uiDevices[selectedIdx].addrType;
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
                    } else {
                        devLabel += " [R]";
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
        bleTrackerLockTarget(pickedName, pickedMac, pickedAddrType);
    }
}

#endif // LITE_VERSION

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
    uint8_t targetAddrType = 0xFF;
    uint16_t headingBucket = 0;
    bool hasTarget = false;
    bool active = false;

    void begin(const String &mac, uint8_t addrType) {
        if (!mutex) mutex = xSemaphoreCreateMutex();
        if (!mutex || !xSemaphoreTake(mutex, portMAX_DELAY)) return;
        NimBLEAddress addr(std::string(mac.c_str()), addrType == 0xFF ? BLE_ADDR_PUBLIC : addrType);
        memcpy(targetMacBytes, addr.getVal(), 6);
        targetAddrType = addrType;
        hasTarget = true;
        head = 0;
        count = 0;
        headingBucket = 0;
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

    void setHeadingBucket(uint16_t bucket) {
        if (!mutex || !xSemaphoreTake(mutex, 0)) return;
        headingBucket = bucket;
        xSemaphoreGive(mutex);
    }

    void appendLocked(int8_t rssi, BleTrackerSampleSource source) {
        samples[head] = {rssi, headingBucket, millis(), source};
        head = (head + 1) % BLE_TRACKER_RING_SIZE;
        if (count < BLE_TRACKER_RING_SIZE) count++;
    }

    void appendTarget(const uint8_t *address, uint8_t addrType, int8_t rssi) {
        if (!mutex) return;
        // Called from the NimBLE host task: never block it - drop the sample if the main
        // loop happens to be reading the buffer at the same instant.
        if (!xSemaphoreTake(mutex, 0)) return;
        if (active && hasTarget && memcmp(address, targetMacBytes, 6) == 0 &&
            (targetAddrType == 0xFF || targetAddrType == addrType)) {
            appendLocked(rssi, TRACK_SRC_ESP32);
        }
        xSemaphoreGive(mutex);
    }

    void appendCurrent(int8_t rssi, BleTrackerSampleSource source) {
        if (!mutex || !xSemaphoreTake(mutex, 0)) return;
        if (active) appendLocked(rssi, source);
        xSemaphoreGive(mutex);
    }

    // Copies buffered samples oldest-first. Samples retain ring order even when timestamps match.
    size_t drain(BleTrackerSample *out, size_t capacity) {
        if (!mutex || !xSemaphoreTake(mutex, 20 / portTICK_PERIOD_MS)) return 0;
        size_t copied = min(count, capacity);
        size_t oldest = (head + BLE_TRACKER_RING_SIZE - count) % BLE_TRACKER_RING_SIZE;
        for (size_t i = 0; i < copied; i++) {
            out[i] = samples[(oldest + i) % BLE_TRACKER_RING_SIZE];
        }
        count -= copied;
        xSemaphoreGive(mutex);
        return copied;
    }
};

BleTrackerHistory g_history;

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
        if (!device) return;
        const NimBLEAddress &address = device->getAddress();
        g_history.appendTarget(address.getVal(), address.getType(), device->getRSSI());
    }
};

TrackerScanCallbacks g_trackerScanCallbacks;

// Direction estimator: per-heading-bin median-of-5 RSSI (with outlier clamping), a peak search on
// a 3-bin smoothed profile and an exponentially sharpened circular centroid around the peak
// (+/-60 deg window, so a back lobe / body shadow can't drag the estimate). Contrast between
// the peak and the median bin and a 180-degree ambiguity check drive the confidence tier.
constexpr float HEADING_BEST_DECAY_DB_PER_SEC = 2.0f;
constexpr float HEADING_NOISE_FLOOR = -100.0f;
constexpr int BIN_WINDOW = 5;
constexpr float BIN_OUTLIER_DB = 12.0f;
constexpr float PEAK_SHARPNESS_DB = 6.0f;

enum HeadingConfidence {
    CONFIDENCE_LOW = 0,  // Red: Need more samples / sweep sectors
    CONFIDENCE_MED = 1,  // Yellow: Gathering sweep data, refining estimate
    CONFIDENCE_HIGH = 2  // Green: High multi-sector confidence
};

struct BestHeadingTable {
    float binRssi[HEADING_BUCKETS];
    float win[HEADING_BUCKETS][BIN_WINDOW];
    uint8_t winN[HEADING_BUCKETS];
    uint8_t winPos[HEADING_BUCKETS];
    uint16_t sampleCount[HEADING_BUCKETS];
    unsigned long lastDecayMs = 0;
    float lastTargetDeg = -1.0f;
    float lastContrastDb = 0.0f;
    bool lastAmbiguous = false;
    bool hasTarget = false;

    void reset() {
        for (int i = 0; i < HEADING_BUCKETS; i++) {
            binRssi[i] = HEADING_NOISE_FLOOR;
            sampleCount[i] = 0;
            winN[i] = 0;
            winPos[i] = 0;
        }
        lastDecayMs = millis();
        lastTargetDeg = -1.0f;
        lastContrastDb = 0.0f;
        lastAmbiguous = false;
        hasTarget = false;
    }

    void decay() {
        unsigned long now = millis();
        float dtSec = (now - lastDecayMs) / 1000.0f;
        lastDecayMs = now;
        if (dtSec <= 0.0f) return;
        float drop = HEADING_BEST_DECAY_DB_PER_SEC * dtSec;
        for (int i = 0; i < HEADING_BUCKETS; i++) {
            if (sampleCount[i] == 0) continue;
            binRssi[i] -= drop;
            for (int k = 0; k < winN[i]; k++) win[i][k] -= drop;
            if (binRssi[i] <= HEADING_NOISE_FLOOR) {
                binRssi[i] = HEADING_NOISE_FLOOR;
                sampleCount[i] = 0;
                winN[i] = 0;
            }
        }
    }

    static float median(const float *v, int n) {
        float t[BIN_WINDOW > 36 ? BIN_WINDOW : 36];
        for (int i = 0; i < n; i++) {
            float x = v[i];
            int j = i - 1;
            while (j >= 0 && t[j] > x) {
                t[j + 1] = t[j];
                j--;
            }
            t[j + 1] = x;
        }
        return (n & 1) ? t[n / 2] : 0.5f * (t[n / 2 - 1] + t[n / 2]);
    }

    void feed(uint16_t bucket, int8_t rssi) {
        if (bucket >= HEADING_BUCKETS) return;
        float r = (float)rssi;
        if (r < HEADING_NOISE_FLOOR) r = HEADING_NOISE_FLOOR;

        if (winN[bucket] >= 3) {
            // Clamp outliers relative to the current bin median instead of letting them jump the bin.
            float med = binRssi[bucket];
            if (r > med + BIN_OUTLIER_DB) r = med + BIN_OUTLIER_DB;
            else if (r < med - BIN_OUTLIER_DB) r = med - BIN_OUTLIER_DB;
        }
        win[bucket][winPos[bucket]] = r;
        winPos[bucket] = (winPos[bucket] + 1) % BIN_WINDOW;
        if (winN[bucket] < BIN_WINDOW) winN[bucket]++;
        binRssi[bucket] = median(win[bucket], winN[bucket]);
        if (sampleCount[bucket] < 100) sampleCount[bucket]++;
    }

    // Pure getter apart from the target low-pass filter: bin ageing is done by decay() in the main loop.
    float getTargetBearingDeg(bool &resolved, HeadingConfidence &conf) {
        int totalSamples = 0;
        int populatedSectors = 0;
        float vals[HEADING_BUCKETS];
        for (int i = 0; i < HEADING_BUCKETS; i++) {
            if (sampleCount[i] > 0 && binRssi[i] > HEADING_NOISE_FLOOR) {
                totalSamples += sampleCount[i];
                vals[populatedSectors++] = binRssi[i];
            }
        }
        conf = CONFIDENCE_LOW;
        if (populatedSectors == 0) {
            resolved = hasTarget;
            return hasTarget ? lastTargetDeg : 0.0f;
        }

        // Peak search on a 3-bin smoothed profile (unpopulated neighbours count as 6 dB below).
        auto bin = [&](int i, float fallback) {
            i = (i % HEADING_BUCKETS + HEADING_BUCKETS) % HEADING_BUCKETS;
            return (sampleCount[i] > 0 && binRssi[i] > HEADING_NOISE_FLOOR) ? binRssi[i] : fallback;
        };
        int peak = -1;
        float peakScore = -1000.0f;
        for (int i = 0; i < HEADING_BUCKETS; i++) {
            if (sampleCount[i] == 0 || binRssi[i] <= HEADING_NOISE_FLOOR) continue;
            float own = binRssi[i];
            float score = 0.5f * own + 0.25f * bin(i - 1, own - 6.0f) + 0.25f * bin(i + 1, own - 6.0f);
            if (score > peakScore) {
                peakScore = score;
                peak = i;
            }
        }
        float rmax = binRssi[peak];
        float contrast = rmax - median(vals, populatedSectors);

        // Sharpened centroid inside +/-60 deg of the peak.
        float sumX = 0.0f, sumY = 0.0f, totalWeight = 0.0f;
        for (int d = -6; d <= 6; d++) {
            int i = ((peak + d) % HEADING_BUCKETS + HEADING_BUCKETS) % HEADING_BUCKETS;
            if (sampleCount[i] == 0 || binRssi[i] <= HEADING_NOISE_FLOOR) continue;
            float countFactor = min((float)sampleCount[i], 4.0f) / 4.0f;
            float w = expf((binRssi[i] - rmax) / PEAK_SHARPNESS_DB) * (0.4f + 0.6f * countFactor);
            float rad = (i * HEADING_BUCKET_DEG) * (M_PI / 180.0f);
            sumX += w * cosf(rad);
            sumY += w * sinf(rad);
            totalWeight += w;
        }

        // 180-degree ambiguity: a second lobe nearly as strong on the opposite side.
        bool ambiguous = false;
        for (int d = -2; d <= 2; d++) {
            int i = ((peak + HEADING_BUCKETS / 2 + d) % HEADING_BUCKETS + HEADING_BUCKETS) % HEADING_BUCKETS;
            if (sampleCount[i] > 0 && binRssi[i] > HEADING_NOISE_FLOOR && binRssi[i] >= rmax - 3.0f) {
                ambiguous = true;
            }
        }
        lastContrastDb = contrast;
        lastAmbiguous = ambiguous;

        if (totalSamples < 6 || populatedSectors < 3) {
            conf = CONFIDENCE_LOW;
        } else if (totalSamples < 14 || populatedSectors < 5 || contrast < 5.0f || ambiguous) {
            conf = CONFIDENCE_MED;
        } else {
            conf = CONFIDENCE_HIGH;
        }

        if (populatedSectors >= 2 && totalSamples >= 3 && contrast >= 3.0f && totalWeight > 0.001f) {
            float targetDeg = atan2f(sumY, sumX) * (180.0f / M_PI);
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

// ---- Walk-and-sweep trilateration -------------------------------------------------------
// The IMU dead-reckons the user's path (heading frame, meters). RSSI samples are averaged into
// ~0.75 m cells along that path; the target position is the point whose log-distance path-loss
// model  rssi = A - 10 n log10(d)  best fits all cells (A is solved in closed form per
// candidate, so the unknown TX power doesn't matter). A coarse-to-fine grid search is used,
// with a weak prior from the direction arrow to break the mirror ambiguity of a straight path.
constexpr int TRI_MAX_CELLS = 64;
constexpr float TRI_CELL_M = 0.75f;
constexpr float PATH_LOSS_N = 2.2f;
constexpr float TX_POWER_AT_1M = -59.0f;

struct RssiCell {
    float x, y;
    float rssi;
    uint16_t n;
    uint32_t lastMs;
};

struct TrilatSolver {
    RssiCell cells[TRI_MAX_CELLS];
    int count = 0;
    bool dirty = false;
    bool resolved = false;
    float estX = 0.0f, estY = 0.0f;
    float rmsDb = 99.0f;
    uint32_t epoch = 0; // bumps whenever the estimate or cell set changes

    void reset() {
        count = 0;
        dirty = false;
        resolved = false;
        rmsDb = 99.0f;
        epoch++;
    }

    void addSample(float x, float y, float rssi, uint32_t ms) {
        int best = -1;
        float bestD2 = TRI_CELL_M * TRI_CELL_M;
        for (int i = 0; i < count; i++) {
            float dx = cells[i].x - x, dy = cells[i].y - y;
            float d2 = dx * dx + dy * dy;
            if (d2 < bestD2) {
                bestD2 = d2;
                best = i;
            }
        }
        if (best >= 0) {
            RssiCell &c = cells[best];
            c.rssi += 0.3f * (rssi - c.rssi);
            if (c.n < 50) c.n++;
            c.lastMs = ms;
        } else {
            int slot = count;
            if (count >= TRI_MAX_CELLS) {
                slot = 0;
                for (int i = 1; i < count; i++)
                    if ((int32_t)(cells[i].lastMs - cells[slot].lastMs) < 0) slot = i;
            } else {
                count++;
            }
            cells[slot] = {x, y, rssi, 1, ms};
        }
        dirty = true;
        epoch++;
    }

    // Cost in dB^2 of a target at (px,py); optionally returns the model-only RMS.
    float cost(float px, float py, bool prior, float curX, float curY, float bearingDeg, float *rmsOut) const {
        float l[TRI_MAX_CELLS];
        float sumW = 0.0f, sumR = 0.0f;
        for (int i = 0; i < count; i++) {
            float dx = cells[i].x - px, dy = cells[i].y - py;
            float d2 = dx * dx + dy * dy + 0.25f; // never closer than 0.5 m
            l[i] = cells[i].rssi + 5.0f * PATH_LOSS_N * log10f(d2);
            float w = min((float)cells[i].n, 4.0f);
            sumW += w;
            sumR += w * l[i];
        }
        float A = sumR / sumW;
        float acc = 0.0f;
        for (int i = 0; i < count; i++) {
            float w = min((float)cells[i].n, 4.0f);
            float e = l[i] - A;
            acc += w * e * e;
        }
        float c = acc / sumW;
        if (rmsOut) *rmsOut = sqrtf(c);
        if (prior) {
            float dx = px - curX, dy = py - curY;
            if (dx * dx + dy * dy > 0.25f) {
                float ang = atan2f(dx, dy) * (180.0f / M_PI);
                c += 25.0f * (1.0f - cosf((ang - bearingDeg) * (M_PI / 180.0f)));
            }
        }
        return c;
    }

    // Returns true if a (new) position estimate was produced.
    bool solve(bool priorValid, float bearingDeg, float curX, float curY) {
        dirty = false;
        if (count < 4) return false;
        float minX = 1e9f, maxX = -1e9f, minY = 1e9f, maxY = -1e9f, minR = 1e9f, maxR = -1e9f;
        for (int i = 0; i < count; i++) {
            minX = min(minX, cells[i].x);
            maxX = max(maxX, cells[i].x);
            minY = min(minY, cells[i].y);
            maxY = max(maxY, cells[i].y);
            minR = min(minR, cells[i].rssi);
            maxR = max(maxR, cells[i].rssi);
        }
        float extent = max(maxX - minX, maxY - minY);
        if (extent < 2.0f || (maxR - minR) < 4.0f) return false;

        float cx = 0.5f * (minX + maxX), cy = 0.5f * (minY + maxY);
        float half = min(20.0f, 0.5f * extent + 12.0f);
        float bx = cx, by = cy, bc = 1e18f;
        for (float py = cy - half; py <= cy + half; py += 1.5f) {
            for (float px = cx - half; px <= cx + half; px += 1.5f) {
                float c = cost(px, py, priorValid, curX, curY, bearingDeg, nullptr);
                if (c < bc) {
                    bc = c;
                    bx = px;
                    by = py;
                }
            }
        }
        const float steps[2] = {0.4f, 0.1f};
        const float spans[2] = {1.5f, 0.4f};
        for (int st = 0; st < 2; st++) {
            float ox = bx, oy = by;
            for (float py = oy - spans[st]; py <= oy + spans[st] + 0.001f; py += steps[st]) {
                for (float px = ox - spans[st]; px <= ox + spans[st] + 0.001f; px += steps[st]) {
                    float c = cost(px, py, priorValid, curX, curY, bearingDeg, nullptr);
                    if (c < bc) {
                        bc = c;
                        bx = px;
                        by = py;
                    }
                }
            }
        }
        float rms = 0.0f;
        cost(bx, by, false, 0, 0, 0, &rms);
        rmsDb = rms;
        if (resolved) {
            estX += 0.6f * (bx - estX);
            estY += 0.6f * (by - estY);
        } else {
            estX = bx;
            estY = by;
        }
        resolved = true;
        epoch++;
        return true;
    }
};

constexpr int CRUMB_MAX = 64;
struct Breadcrumbs {
    float x[CRUMB_MAX], y[CRUMB_MAX];
    int count = 0;
    int head = 0;

    void reset() { count = head = 0; }

    void add(float px, float py) {
        if (count > 0) {
            int last = (head + CRUMB_MAX - 1) % CRUMB_MAX;
            float dx = px - x[last], dy = py - y[last];
            if (dx * dx + dy * dy < 0.35f * 0.35f) return;
        }
        x[head] = px;
        y[head] = py;
        head = (head + 1) % CRUMB_MAX;
        if (count < CRUMB_MAX) count++;
    }

    int idx(int i) const { return (head + CRUMB_MAX - count + i) % CRUMB_MAX; } // i = 0 oldest
};

// Distance from smoothed RSSI using the log-distance model (rough guide only).
float estimateDistanceM(float rssi) {
    return powf(10.0f, (TX_POWER_AT_1M - rssi) / (10.0f * PATH_LOSS_N));
}

// Draws an arrow centered at (cx, cy) with the given radius, pointing `angleDeg` clockwise
// from straight up (0deg = up, matching a compass rose drawn on screen).
template <class G> void drawHeadingArrow(G &g, int cx, int cy, int radius, float angleDeg, uint16_t color) {
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

    g.fillTriangle(tipX, tipY, leftX, leftY, rightX, rightY, color);
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

// Little north-up (initial-facing-up) map of the walked path, RSSI cells, the user and the
// trilaterated target. Scales automatically to fit everything with a minimum 8 m span.
template <class G> void drawTrackMapOn(
    G &g, int mx, int my, int msz, const Breadcrumbs &crumbs, const TrilatSolver &tri, float curX, float curY,
    float headingDeg, bool stale
) {
    float minX = curX, maxX = curX, minY = curY, maxY = curY;
    auto grow = [&](float x, float y) {
        minX = min(minX, x);
        maxX = max(maxX, x);
        minY = min(minY, y);
        maxY = max(maxY, y);
    };
    for (int i = 0; i < crumbs.count; i++) {
        int k = crumbs.idx(i);
        grow(crumbs.x[k], crumbs.y[k]);
    }
    if (tri.resolved) grow(tri.estX, tri.estY);
    float span = max(max(maxX - minX, maxY - minY) + 2.0f, 8.0f);
    float cx = 0.5f * (minX + maxX), cy = 0.5f * (minY + maxY);
    float scale = (msz - 8) / span;
    int midX = mx + msz / 2, midY = my + msz / 2;
    auto px = [&](float x) { return midX + (int)lroundf((x - cx) * scale); };
    auto py = [&](float y) { return midY - (int)lroundf((y - cy) * scale); };
    auto clampIn = [&](int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); };

    g.fillRect(mx, my, msz, msz, bruceConfig.bgColor);
    g.drawRect(mx, my, msz, msz, TFT_DARKGREY);

    int lx = 0, ly = 0;
    for (int i = 0; i < crumbs.count; i++) {
        int k = crumbs.idx(i);
        int x = clampIn(px(crumbs.x[k]), mx + 1, mx + msz - 2);
        int y = clampIn(py(crumbs.y[k]), my + 1, my + msz - 2);
        if (i > 0) g.drawLine(lx, ly, x, y, TFT_DARKGREY);
        lx = x;
        ly = y;
    }
    for (int i = 0; i < tri.count; i++) {
        int x = clampIn(px(tri.cells[i].x), mx + 1, mx + msz - 3);
        int y = clampIn(py(tri.cells[i].y), my + 1, my + msz - 3);
        float f = constrain((tri.cells[i].rssi + 100.0f) / 60.0f, 0.0f, 1.0f);
        g.fillRect(x, y, 2, 2, getVuColor(f));
    }
    if (tri.resolved && !stale) {
        int tx = clampIn(px(tri.estX), mx + 4, mx + msz - 5);
        int ty = clampIn(py(tri.estY), my + 4, my + msz - 5);
        uint16_t col = tri.rmsDb < 6.0f ? TFT_RED : tft.color565(230, 150, 40);
        g.drawCircle(tx, ty, 3, col);
        g.drawLine(tx - 2, ty - 2, tx + 2, ty + 2, col);
        g.drawLine(tx - 2, ty + 2, tx + 2, ty - 2, col);
    }
    int ux = clampIn(px(curX), mx + 2, mx + msz - 3);
    int uy = clampIn(py(curY), my + 2, my + msz - 3);
    float rad = headingDeg * (PI / 180.0f);
    g.drawLine(ux, uy, ux + (int)lroundf(sinf(rad) * 6), uy - (int)lroundf(cosf(rad) * 6), TFT_CYAN);
    g.fillCircle(ux, uy, 2, TFT_WHITE);
}

void drawTrackMap(
    int mx, int my, int msz, const Breadcrumbs &crumbs, const TrilatSolver &tri, float curX, float curY,
    float headingDeg, bool stale
) {
    // Compose off-screen and push in one transfer so the map never visibly clears (no flicker).
    tft_sprite spr(static_cast<tft_display *>(&tft));
    spr.setColorDepth(16);
    if (spr.createSprite(msz, msz) != nullptr) {
        drawTrackMapOn(spr, 0, 0, msz, crumbs, tri, curX, curY, headingDeg, stale);
        spr.pushSprite(mx, my);
        spr.deleteSprite();
    } else {
        drawTrackMapOn(tft, mx, my, msz, crumbs, tri, curX, curY, headingDeg, stale);
    }
}

} // namespace

void bleTrackerLockTarget(const String &label, const String &mac, uint8_t addrType) {
    while (true) {
        bool isFav = bruceConfig.isBleTrackerFavorite(mac, addrType);
        std::vector<Option> targetOptions = {
            {"Track now", [=]() { bleTrackerRun(mac, label, nullptr, addrType); }},
        };
        if (isFav) {
            targetOptions.push_back({"Remove from favorites", [=]() {
                bruceConfig.removeBleTrackerFavorite(mac, addrType);
                displaySuccess("Removed from favs", true);
            }});
        } else {
            targetOptions.push_back({"Save as favorite & track", [=]() {
                String saved = promptForLabel(label);
                bruceConfig.addBleTrackerFavorite(saved, mac, addrType);
                bleTrackerRun(mac, saved, nullptr, addrType);
            }});
        }
        targetOptions.push_back({"< Back", []() {}});

        int chosen = loopOptions(targetOptions, MENU_TYPE_SUBMENU, "Target Options");
        if (chosen < 0 || chosen == (int)targetOptions.size() - 1) {
            break;
        }
    }
}

void bleTrackerScanAndPick() {
#if !defined(LITE_VERSION)
    String name;
    String mac;
    int rssi = -100;
    uint8_t addrType = BLE_ADDR_PUBLIC;
    if (gattScanAndPick(name, mac, rssi, addrType, true, true)) {
        bleTrackerLockTarget(name, mac, addrType);
    }
#else
    bleTrackerPickFromScan();
#endif
}

void BleTrackerMenu() {
    while (true) {
        std::vector<Option> trackerOptions;

        for (const auto &fav : bruceConfig.bleTrackerFavorites) {
            String label = fav.label;
            String mac = fav.mac;
            uint8_t addrType = fav.addrType;
            trackerOptions.emplace_back(label, [=]() { bleTrackerLockTarget(label, mac, addrType); });
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

void bleTrackerRun(const String &targetMac, const String &label, NimBLEClient *pClient, uint8_t targetAddrType,
                   uint16_t connectedServerHandle) {
    // Full-screen tracking view, sized for 240x135 (Cardputer/ADV) and smaller displays: a
    // title row with [ACTIVE]/[PASSIVE] status indicator, an audio-mixer style VU meter with
    // smoothed RSSI & decaying peak hold, and a dBm/stale readout. A persistent onResult()
    // callback (TrackerScanCallbacks) keeps feeding the ring buffer in the background, while
    // existing client/server connections can poll link-layer RSSI directly.
    bool bleWasActiveBefore = BLEConnected || (BLEDevice::getServer() != nullptr);
#if !defined(LITE_VERSION)
    bleWasActiveBefore =
        bleWasActiveBefore || BLEStateManager::isBLEActive() || BLEStateManager::getActiveClientCount() > 0;
#endif

    bool scannerWasPresent = pBLEScan != nullptr;
    bool scannerWasScanning = scannerWasPresent && pBLEScan->isScanning();
    bool ownsScanner = !bleWasActiveBefore && !scannerWasPresent && !scannerWasScanning;
    bool connectedClient = pClient != nullptr && pClient->isConnected();
    bool connectedServerPeer = connectedServerHandle != 0xFFFF;
    if (!ownsScanner && !connectedClient && !connectedServerPeer) {
        displayError("BLE scanner already in use");
        return;
    }

    if (ownsScanner) {
        if (!ble_scan_setup() || pBLEScan == nullptr) {
            displayError("Failed to init BLE scan");
            return;
        }
        pBLEScan->setActiveScan(false);                           // passive: never send scan requests
        pBLEScan->setScanCallbacks(&g_trackerScanCallbacks, true); // wantDuplicates=true: keep seeing the same MAC
    }

    g_history.begin(targetMac, targetAddrType);

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
    }
    BestHeadingTable bestHeading;
    if (hasImu) bestHeading.reset();
    TrilatSolver trilat;
    trilat.reset();
    Breadcrumbs crumbs;
    crumbs.reset();
    float curX = 0.0f, curY = 0.0f;
    float trendHist[8] = {0};
    int trendCount = 0;
    unsigned long lastTrendMs = 0, lastSolveMs = 0, lastMapMs = 0;
    int lastTrendState = -9, lastDistKey = -9;
    uint32_t lastMapEpoch = 0xFFFFFFFF, lastMapSteps = 0xFFFFFFFF;
    float lastMapHeading = -999.0f;

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
    // Position map right of the arrow, same height; text sits between arrow and map.
    int mapSize = 2 * arrowRadius + 4;
    int mapX = tftWidth - BORDER_PAD_X - mapSize;
    int mapY = arrowCenterY - mapSize / 2;

    float emaRssi = -100.0f;
    bool emaInitialized = false;
    unsigned long lastSeenMs = 0;

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

    if (ownsScanner && !pBLEScan->start(0, false)) { // duration=0: scan indefinitely until stop()
        g_history.end();
        pBLEScan->setScanCallbacks(nullptr);
        stopBLEStack();
        displayError("Failed to start BLE scan");
        return;
    }

    while (!check(EscPress)) {
        unsigned long nowMs = millis();
        float dtSec = (nowMs - lastFrameMs) / 1000.0f;
        if (dtSec < 0.0f || dtSec > 1.0f) dtSec = 0.05f;
        lastFrameMs = nowMs;

        // Poll RSSI directly on an existing client or honeypot server connection.
        bool isActivelyConnected = false;
        if (pClient != nullptr && pClient->isConnected()) {
            int connRssi = pClient->getRssi();
            if (connRssi != 0) {
                g_history.appendCurrent((int8_t)connRssi, TRACK_SRC_ESP32);
                isActivelyConnected = true;
            }
        } else if (connectedServerPeer) {
            int8_t connRssi = 0;
            if (ble_gap_conn_rssi(connectedServerHandle, &connRssi) == 0) {
                g_history.appendCurrent(connRssi, TRACK_SRC_ESP32);
                isActivelyConnected = true;
            }
        }

        if (hasImu) {
            currentHeadingDeg = imu_get_heading_delta_deg();
            imu_get_position(curX, curY);
            crumbs.add(curX, curY);
            bestHeading.decay();
        }

        static BleTrackerSample pendingSamples[BLE_TRACKER_RING_SIZE];
        size_t pendingCount = g_history.drain(pendingSamples, BLE_TRACKER_RING_SIZE);
        for (size_t i = 0; i < pendingCount; i++) {
            const BleTrackerSample &sample = pendingSamples[i];
            constexpr float EMA_ALPHA = 0.25f;
            emaRssi = emaInitialized ? (EMA_ALPHA * sample.rssi + (1.0f - EMA_ALPHA) * emaRssi) : (float)sample.rssi;
            emaInitialized = true;
            lastSeenMs = sample.timestamp;

            float rawFraction = constrain(((float)sample.rssi - (-100.0f)) / 60.0f, 0.0f, 1.0f);
            if (rawFraction >= peakFraction) {
                peakFraction = rawFraction;
                peakHoldUntilMs = nowMs + 800; // Hold peak for 800ms
            }
            if (hasImu) {
                // Tag with the heading interpolated at the packet's own timestamp.
                bestHeading.feed(headingDegToBucket(imu_heading_at(sample.timestamp)), sample.rssi);
                trilat.addSample(curX, curY, (float)sample.rssi, sample.timestamp);
            }
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
                tft.printf("%d%% (%d dBm)  Peak: %d dBm", pct, (int)lroundf(emaRssi), peakDbm);
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

            // Hot/cold trend: linear fit over the last ~3 s of smoothed RSSI (sampled every 500 ms).
            if (stale) {
                trendCount = 0;
            } else if (nowMs - lastTrendMs >= 500) {
                lastTrendMs = nowMs;
                if (trendCount == 8) {
                    for (int i = 1; i < 8; i++) trendHist[i - 1] = trendHist[i];
                    trendCount = 7;
                }
                trendHist[trendCount++] = emaRssi;
            }
            int trendState = 0; // -1 colder, 0 steady, +1 warmer, 2 = unknown
            if (trendCount < 5) {
                trendState = 2;
            } else {
                float sx = 0, sy = 0, sxx = 0, sxy = 0;
                for (int i = 0; i < trendCount; i++) {
                    sx += i;
                    sy += trendHist[i];
                    sxx += i * i;
                    sxy += i * trendHist[i];
                }
                float denom = trendCount * sxx - sx * sx;
                float slope = denom > 0 ? (trendCount * sxy - sx * sy) / denom : 0.0f; // dB per 0.5 s
                float change = slope * (trendCount - 1);
                trendState = change > 1.5f ? 1 : (change < -1.5f ? -1 : 0);
            }
            float distM = estimateDistanceM(emaRssi);
            int distKey = stale ? -1 : (distM < 3.0f ? (int)(distM * 2.0f) : 10 + (int)distM);

            // Redraw if relative angle rotated by at least 2 degrees, or resolution status changed,
            // or confidence changed, or staleness changed, or first frame.
            if (firstDraw || abs(angleDeg - lastDrawnAngleDeg) >= 2 || resolved != lastDrawnResolved ||
                conf != lastDrawnConf || stale != lastDrawnArrowStale || trendState != lastTrendState ||
                distKey != lastDistKey) {
                lastTrendState = trendState;
                lastDistKey = distKey;

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

                {
                    // Compass is composed off-screen and pushed once -> no erase/redraw flicker.
                    int side = 2 * arrowRadius + 6;
                    int ox = arrowCenterX - side / 2, oy = arrowCenterY - side / 2;
                    tft_sprite cs(static_cast<tft_display *>(&tft));
                    cs.setColorDepth(16);
                    bool useSpr = cs.createSprite(side, side) != nullptr;
                    if (useSpr) {
                        int c = side / 2;
                        cs.fillRect(0, 0, side, side, bruceConfig.bgColor);
                        cs.fillCircle(c, c, arrowRadius, bgCol);
                        cs.drawCircle(c, c, arrowRadius, ringCol);
                        if (resolved) {
                            drawHeadingArrow(cs, c, c, arrowRadius - 3, (float)angleDeg, arrowCol);
                        } else {
                            cs.drawPixel(c, c, arrowCol);
                            cs.drawPixel(c - 1, c, arrowCol);
                            cs.drawPixel(c + 1, c, arrowCol);
                            cs.drawPixel(c, c - 1, arrowCol);
                            cs.drawPixel(c, c + 1, arrowCol);
                        }
                        cs.pushSprite(ox, oy);
                        cs.deleteSprite();
                    } else {
                        tft.fillCircle(arrowCenterX, arrowCenterY, arrowRadius + 2, bruceConfig.bgColor);
                        tft.fillCircle(arrowCenterX, arrowCenterY, arrowRadius, bgCol);
                        tft.drawCircle(arrowCenterX, arrowCenterY, arrowRadius, ringCol);
                        if (resolved) drawHeadingArrow(tft, arrowCenterX, arrowCenterY, arrowRadius - 3, (float)angleDeg, arrowCol);
                    }
                }

                // Render side text cleanly separated from compass rose
                int textW = mapX - 4 - infoTextX;
                if (textW > 20) {
                    int line1Y = arrowCenterY - arrowRadius + 2;
                    int line2Y = line1Y + LH * FP + 3;
                    int line3Y = line2Y + LH * FP + 3;
                    tft.fillRect(infoTextX, line1Y, textW, LH * FP * 3 + 10, bruceConfig.bgColor);

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
                    if (!stale) {
                        tft.setCursor(infoTextX, line3Y);
                        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
                        if (distM < 10.0f) tft.printf("~%.1fm ", distM);
                        else tft.printf("~%dm ", (int)distM);
                        if (trendState == 1) {
                            tft.setTextColor(tft.color565(60, 220, 80), bruceConfig.bgColor);
                            tft.print("Warmer");
                        } else if (trendState == -1) {
                            tft.setTextColor(tft.color565(240, 80, 80), bruceConfig.bgColor);
                            tft.print("Colder");
                        } else if (trendState == 0) {
                            tft.setTextColor(TFT_DARKGREY, bruceConfig.bgColor);
                            tft.print("Steady");
                        }
                    }
                    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
                }

                lastDrawnAngleDeg = angleDeg;
                lastDrawnResolved = resolved;
                lastDrawnConf = conf;
                lastDrawnArrowStale = stale;
            }

            // Trilateration: refit when new data arrived (at most every 1.5 s), then redraw the map
            // when the path, estimate or heading tick changed (at most ~7 fps).
            if (trilat.dirty && nowMs - lastSolveMs >= 1500) {
                lastSolveMs = nowMs;
                bool priorOk = resolved && conf >= CONFIDENCE_MED;
                trilat.solve(priorOk, targetDeg, curX, curY);
            }
            uint32_t steps = imu_step_count();
            if (firstDraw || (nowMs - lastMapMs >= 150 &&
                              (trilat.epoch != lastMapEpoch || steps != lastMapSteps ||
                               fabsf(currentHeadingDeg - lastMapHeading) >= 6.0f))) {
                lastMapMs = nowMs;
                lastMapEpoch = trilat.epoch;
                lastMapSteps = steps;
                lastMapHeading = currentHeadingDeg;
                drawTrackMap(mapX, mapY, mapSize, crumbs, trilat, curX, curY, currentHeadingDeg, stale);
            }
        }

        firstDraw = false;
        vTaskDelay(pdMS_TO_TICKS(15)); // fast loop keeps IMU integration accurate; UI redraws are gated
    }

    g_history.end();
    if (ownsScanner && pBLEScan) {
        pBLEScan->stop();
        pBLEScan->clearResults();
        pBLEScan->setScanCallbacks(nullptr);
    }

    if (pClient == nullptr && !connectedServerPeer && !bleWasActiveBefore) {
#if !defined(LITE_VERSION)
        if (!BLEStateManager::isBLEActive()) stopBLEStack();
#else
        stopBLEStack();
#endif
    }
}
