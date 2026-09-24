#if !defined(LITE_VERSION)

#include "gatt_explorer.h"
#include "gatt_server.h"
#include "race_client.h"
#include "BLE_Suite.h"
#include "ble_common.h"
#include "ble_oui.h"
#include "ble_tracker.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/radio_mem.h"
#include "core/sd_functions.h"
#include "core/utils.h"
#include <LittleFS.h>
#include <NimBLEDevice.h>
#include <SD.h>
#include <globals.h>
#include <algorithm>
#include <functional>
#include <vector>

//=============================================================================
// Types & Enums
//=============================================================================

enum GattFilterMode {
    FILTER_CONNECTABLE = 0,
    FILTER_WITH_SERVICES,
    FILTER_HID,
    FILTER_UART,
    FILTER_AUDIO,
    FILTER_SENSORS,
    FILTER_CUSTOM_128,
};

struct GattSettings {
    int minRssi = -100;        // -100 (All), -85, -75, -65
    int timeoutSec = 3;       // 3, 5, 8, 12, 15
    int scanRespTimeout = 50;  // 0 (Off), 50, 100, 250, 500 ms
    bool includeOnDiscovered = true; // false (onResult only), true (include onDiscovered)
    int addrTypeFilter = 0;   // 0: Any, 1: Public only, 2: Random only
    int maxDevices = 60;      // Ring buffer capacity: 20, 40, 60, 80
};

struct GattScannedDevice {
    NimBLEAddress address;
    uint8_t addressType = BLE_ADDR_PUBLIC;
    uint8_t macBytes[6] = {0};
    char macStr[18] = {0};
    char name[32] = {0};
    char vendor[24] = {0};
    char tag[8] = "GATT";
    int8_t rssi = -100;
    bool isConnectable = true;
    uint32_t lastSeen = 0;
    uint16_t packetCount = 0;
    uint16_t srv16[4] = {0};
    uint8_t srv16Count = 0;
    bool has128b = false;
};

//=============================================================================
// State
//=============================================================================

constexpr size_t GATT_MAX_SCAN_DEVICES = 40;

struct GattScannerState {
    GattScannedDevice devices[GATT_MAX_SCAN_DEVICES];
    size_t count = 0;
    uint32_t totalPackets = 0;
    SemaphoreHandle_t mutex = nullptr;
    volatile bool active = false;
    GattFilterMode filterMode = FILTER_CONNECTABLE;

    void reset(GattFilterMode filter) {
        if (!mutex) mutex = xSemaphoreCreateMutex();
        if (mutex && xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
            count = 0;
            totalPackets = 0;
            filterMode = filter;
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

static GattSettings g_gattSettings;
static GattScannerState g_gattScanState;
static std::vector<GattScannedDevice> g_discoveredDevices;
static volatile bool g_scanActive = false;
static GattFilterMode g_currentFilter = FILTER_CONNECTABLE;

// When set (via gattScanAndPick()), the live scanner hands the selected device to
// this callback instead of entering exploreGattDevice()'s GATT connect/browse flow - lets
// other features (e.g. the BLE Tracker) reuse this screen purely as a device picker.
static std::function<void(const GattScannedDevice &)> g_gattPickCallback = nullptr;

//=============================================================================
// UI Layout & Helper Functions (Small Font / Compact Mode)
//=============================================================================

struct GattUiGeom {
    int listL, listW;
    int top;
    int rowH;
    int rows;
    int footY;
};

static GattUiGeom gattUiGeom() {
    GattUiGeom g;
    g.listL = 8;
    g.listW = tftWidth - 16;
    g.top = BORDER_PAD_Y + 8 * FM + 3;
    g.footY = tftHeight - 8 * FP - 6;
    g.rowH = 8 * FP + 5;
    int avail = g.footY - g.top - 2;
    if (avail < g.rowH) avail = g.rowH;
    g.rows = avail / g.rowH;
    if (g.rows < 1) g.rows = 1;
    return g;
}

static uint16_t gattDimColor() {
    return getColorVariation(bruceConfig.priColor, 8, -1);
}

String gattFitText(const String &text, int maxPx) {
    if (maxPx <= 0) return "";
    tft.setTextSize(FP);
    if (tft.textWidth(text.c_str()) <= maxPx) return text;
    String s = text;
    while (s.length() > 1 && tft.textWidth((s + "..").c_str()) > maxPx) {
        s.remove(s.length() - 1);
    }
    return s + "..";
}

static void gattWrapInto(const String &text, int w, std::vector<String> &out) {
    tft.setTextSize(FP);
    const int len = text.length();
    if (len == 0) {
        out.push_back("");
        return;
    }
    int start = 0;
    while (start < len) {
        int end = start, lastSpace = -1;
        while (end < len) {
            if (text.charAt(end) == ' ') lastSpace = end;
            if (tft.textWidth(text.substring(start, end + 1).c_str()) > w) break;
            end++;
        }
        int cut = (end >= len) ? len : (lastSpace > start ? lastSpace : end);
        out.push_back(text.substring(start, cut));
        start = (cut < len && text.charAt(cut) == ' ') ? cut + 1 : cut;
    }
}

void gattDrawRssi(int x, int y, int rssi, uint16_t color) {
    int bars = 0;
    if (rssi > -55) bars = 4;
    else if (rssi > -68) bars = 3;
    else if (rssi > -80) bars = 2;
    else if (rssi > -92) bars = 1;
    for (int i = 0; i < 4; i++) {
        int h = 2 + i * 2;
        if (i < bars) tft.fillRect(x + i * 3, y + 8 - h, 2, h, color);
        else tft.drawFastHLine(x + i * 3, y + 7, 2, color);
    }
}

typedef std::function<void(int idx, int x, int y, int w, bool selected)> GattRowDrawer;

int gattListLoop(
    const char *title, int count, const String &hint, GattRowDrawer drawRow, int *cursor
) {
    if (count <= 0) return -1;
    GattUiGeom g = gattUiGeom();
    int sel = (cursor && *cursor >= 0 && *cursor < count) ? *cursor : 0;
    int off = 0, lastSel = -1, lastOff = -1;

    if (sel >= g.rows) off = sel - g.rows + 1;
    if (title && strlen(title) > 0) {
        drawMainBorderWithTitle(title);
    } else {
        drawMainBorder(true);
    }

    for (;;) {
        if (sel != lastSel || off != lastOff) {
            tft.setTextSize(FP);
            for (int i = 0; i < g.rows; i++) {
                int y = g.top + i * g.rowH;
                int idx = off + i;
                bool selected = (idx == sel);
                tft.fillRect(
                    g.listL,
                    y - 2,
                    g.listW,
                    g.rowH,
                    selected ? bruceConfig.priColor : bruceConfig.bgColor
                );
                if (idx < count) drawRow(idx, g.listL + 3, y, g.listW - 6, selected);
            }

            // Position & hint footer
            tft.fillRect(g.listL, g.footY, g.listW, 8 * FP, bruceConfig.bgColor);
            tft.setTextSize(FP);
            String pos = String(sel + 1) + "/" + String(count);
            int posW = tft.textWidth(pos.c_str());
            tft.setTextColor(gattDimColor(), bruceConfig.bgColor);
            tft.drawString(gattFitText(hint, g.listW - posW - 8), g.listL, g.footY, 1);
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawRightString(pos, g.listL + g.listW, g.footY, 1);

            lastSel = sel;
            lastOff = off;
            if (cursor) *cursor = sel;
        }

        if (check(EscPress)) return -1;
        else if (check(PrevPress) || check(UpPress)) sel = (sel > 0) ? sel - 1 : count - 1;
        else if (check(NextPress) || check(DownPress)) sel = (sel < count - 1) ? sel + 1 : 0;
        else if (check(SelPress)) {
            if (cursor) *cursor = sel;
            return sel;
        }

        if (sel < off) off = sel;
        if (sel >= off + g.rows) off = sel - g.rows + 1;
        if (off > count - g.rows) off = std::max(0, count - g.rows);
        if (off < 0) off = 0;
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

struct GattMenuItem {
    String label;
    std::function<void()> action;
};

static int gattMenu(
    const char *title,
    const std::vector<GattMenuItem> &items,
    const String &hint = "SEL choose  ESC back",
    int *cursor = nullptr
) {
    if (items.empty()) return -1;
    int count = items.size();
    auto drawer = [&items](int idx, int x, int y, int w, bool sel) {
        uint16_t fg = sel ? bruceConfig.bgColor : bruceConfig.priColor;
        uint16_t bg = sel ? bruceConfig.priColor : bruceConfig.bgColor;
        tft.setTextColor(fg, bg);
        tft.setTextSize(FP);
        tft.drawString(gattFitText(items[idx].label, w), x, y, 1);
    };

    int chosen = gattListLoop(title, count, hint, drawer, cursor);
    if (chosen >= 0 && chosen < count) {
        if (items[chosen].action) {
            items[chosen].action();
        }
    }
    return chosen;
}

static void gattShowScrollableReport(const char *title, const std::vector<String> &lines) {
    GattUiGeom g = gattUiGeom();
    const int barW = 3;
    const int textX = g.listL + barW + 4;
    const int textW = g.listW - barW - 4;

    std::vector<String> rows;
    for (size_t i = 0; i < lines.size(); i++) {
        gattWrapInto(lines[i], textW, rows);
    }

    const int perPage = g.rows;
    int off = 0, lastOff = -1;

    drawMainBorderWithTitle(title);
    tft.fillRect(g.listL, g.top - 2, barW, perPage * g.rowH, bruceConfig.priColor);

    for (;;) {
        if (off != lastOff) {
            lastOff = off;
            tft.setTextSize(FP);
            for (int i = 0; i < perPage; i++) {
                int y = g.top + i * g.rowH;
                tft.fillRect(textX, y - 2, textW, g.rowH, bruceConfig.bgColor);
                size_t idx = (size_t)(off + i);
                if (idx >= rows.size()) continue;
                tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
                tft.drawString(rows[idx], textX, y, 1);
            }

            tft.fillRect(g.listL, g.footY, g.listW, 8 * FP, bruceConfig.bgColor);
            bool more = (int)rows.size() > perPage;
            tft.setTextColor(gattDimColor(), bruceConfig.bgColor);
            tft.drawString(more ? "UP/DN scroll  ESC/SEL back" : "SEL / ESC back", g.listL, g.footY, 1);
            if (more) {
                int last = std::min((int)rows.size(), off + perPage);
                String pos = String(off + 1) + "-" + String(last) + "/" + String((int)rows.size());
                tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
                tft.drawRightString(pos, g.listL + g.listW, g.footY, 1);
            }
        }

        if (check(EscPress) || check(SelPress)) return;
        else if (check(PrevPress) || check(UpPress)) off = std::max(0, off - 1);
        else if (check(NextPress) || check(DownPress)) {
            if (off + perPage < (int)rows.size()) off++;
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

//=============================================================================
// Helper: UUID Name Resolution
//=============================================================================

static String getGattServiceName(const String &uuidStr) {
    String lower = uuidStr;
    lower.toLowerCase();
    if (lower.startsWith("0x")) {
        lower = lower.substring(2);
    }

    // Standard 16-bit Service UUIDs (or 16-bit inside standard Bluetooth Base UUID)
    if (lower == "1800" || lower.indexOf("00001800-0000-1000-8000-00805f9b34fb") != -1) return "Generic Access";
    if (lower == "1801" || lower.indexOf("00001801-0000-1000-8000-00805f9b34fb") != -1) return "Generic Attribute";
    if (lower == "180a" || lower.indexOf("0000180a-0000-1000-8000-00805f9b34fb") != -1) return "Device Info";
    if (lower == "180f" || lower.indexOf("0000180f-0000-1000-8000-00805f9b34fb") != -1) return "Battery Service";
    if (lower == "1812" || lower.indexOf("00001812-0000-1000-8000-00805f9b34fb") != -1) return "HID (Human Interface)";
    if (lower == "180d" || lower.indexOf("0000180d-0000-1000-8000-00805f9b34fb") != -1) return "Heart Rate";
    if (lower == "181a" || lower.indexOf("0000181a-0000-1000-8000-00805f9b34fb") != -1) return "Environmental";
    if (lower == "1809" || lower.indexOf("00001809-0000-1000-8000-00805f9b34fb") != -1) return "Health Thermometer";
    if (lower == "1808" || lower.indexOf("00001808-0000-1000-8000-00805f9b34fb") != -1) return "Glucose";
    if (lower == "1810" || lower.indexOf("00001810-0000-1000-8000-00805f9b34fb") != -1) return "Blood Pressure";
    if (lower == "1815" || lower.indexOf("00001815-0000-1000-8000-00805f9b34fb") != -1) return "Automation IO";
    if (lower == "1816" || lower.indexOf("00001816-0000-1000-8000-00805f9b34fb") != -1) return "Cycling Speed";
    if (lower == "1818" || lower.indexOf("00001818-0000-1000-8000-00805f9b34fb") != -1) return "Cycling Power";
    if (lower == "1826" || lower.indexOf("00001826-0000-1000-8000-00805f9b34fb") != -1) return "Fitness Machine";
    if (lower == "1843" || lower.indexOf("00001843-0000-1000-8000-00805f9b34fb") != -1) return "Audio Input Control";
    if (lower == "1844" || lower.indexOf("00001844-0000-1000-8000-00805f9b34fb") != -1) return "Volume Control";
    if (lower == "110a" || lower.indexOf("0000110a-0000-1000-8000-00805f9b34fb") != -1) return "A2DP Audio Sink";
    if (lower == "110b" || lower.indexOf("0000110b-0000-1000-8000-00805f9b34fb") != -1) return "A2DP Source";
    if (lower == "110e" || lower.indexOf("0000110e-0000-1000-8000-00805f9b34fb") != -1) return "AVRCP Target";
    if (lower == "110f" || lower.indexOf("0000110f-0000-1000-8000-00805f9b34fb") != -1) return "AVRCP Controller";
    if (lower == "111e" || lower.indexOf("0000111e-0000-1000-8000-00805f9b34fb") != -1) return "Handsfree (HFP)";
    if (lower == "fe2c" || lower.indexOf("0000fe2c-0000-1000-8000-00805f9b34fb") != -1) return "Google FastPair";
    if (lower == "fee0" || lower.indexOf("0000fee0-0000-1000-8000-00805f9b34fb") != -1) return "Mi Band / Smart";
    if (lower == "ffe0" || lower.indexOf("0000ffe0-0000-1000-8000-00805f9b34fb") != -1) return "HM-10 / UART";
    if (lower == "fef0" || lower.indexOf("0000fef0-0000-1000-8000-00805f9b34fb") != -1) return "Airoha FOTA / UART";
    if (lower.indexOf("6e400001") != -1) return "Nordic NUS (UART)";
    if (lower.indexOf("5052494d-2dab-0341-6972-6f6861424c45") != -1) return "Airoha RACE (BLE)";
    if (lower.indexOf("dc405470-a351-4a59-97d8-2e2e3b207fbb") != -1) return "Sony RACE / Audio";
    if (lower.indexOf("49535343-fe7d-4ae5-8fa9-9fafd205e455") != -1) return "TRSPX Transparent";
    if (lower.indexOf("00000000-0000-0000-0099-aabbccddeeff") != -1) return "Airoha Classic SPP";
    if (lower.indexOf("8901dfa8-5c7e-4d8f-9f0c-c2b70683f5f0") != -1) return "Sony Classic RACE";
    if (lower.indexOf("2d064aa9-32b5-4970-865c-643742bd2862") != -1) return "Bose Classic RACE";
    if (lower.indexOf("0000fff0") != -1) return "Vendor Serial";

    if (lower.length() > 8) return "Custom 128-bit";
    return "Service 0x" + lower;
}

static String getGattCharName(const String &uuidStr) {
    String lower = uuidStr;
    lower.toLowerCase();
    if (lower.startsWith("0x")) {
        lower = lower.substring(2);
    }

    if (lower == "2a00" || lower.indexOf("00002a00-") != -1) return "Device Name";
    if (lower == "2a01" || lower.indexOf("00002a01-") != -1) return "Appearance";
    if (lower == "2a04" || lower.indexOf("00002a04-") != -1) return "Conn Params";
    if (lower == "2a05" || lower.indexOf("00002a05-") != -1) return "Service Changed";
    if (lower == "2a19" || lower.indexOf("00002a19-") != -1) return "Battery Level";
    if (lower == "2a1c" || lower.indexOf("00002a1c-") != -1) return "Temperature Meas";
    if (lower == "2a23" || lower.indexOf("00002a23-") != -1) return "System ID";
    if (lower == "2a24" || lower.indexOf("00002a24-") != -1) return "Model Number";
    if (lower == "2a25" || lower.indexOf("00002a25-") != -1) return "Serial Number";
    if (lower == "2a26" || lower.indexOf("00002a26-") != -1) return "Firmware Rev";
    if (lower == "2a27" || lower.indexOf("00002a27-") != -1) return "Hardware Rev";
    if (lower == "2a28" || lower.indexOf("00002a28-") != -1) return "Software Rev";
    if (lower == "2a29" || lower.indexOf("00002a29-") != -1) return "Manufacturer";
    if (lower == "2a2a" || lower.indexOf("00002a2a-") != -1) return "Regulatory Cert";
    if (lower == "2a37" || lower.indexOf("00002a37-") != -1) return "Heart Rate Meas";
    if (lower == "2a4a" || lower.indexOf("00002a4a-") != -1) return "HID Info";
    if (lower == "2a4b" || lower.indexOf("00002a4b-") != -1) return "Report Map";
    if (lower == "2a4c" || lower.indexOf("00002a4c-") != -1) return "HID Ctrl Point";
    if (lower == "2a4d" || lower.indexOf("00002a4d-") != -1) return "HID Report";
    if (lower == "2a4e" || lower.indexOf("00002a4e-") != -1) return "Protocol Mode";
    if (lower == "2a50" || lower.indexOf("00002a50-") != -1) return "PnP ID";
    if (lower == "2a6e" || lower.indexOf("00002a6e-") != -1) return "Temperature";
    if (lower == "2a6f" || lower.indexOf("00002a6f-") != -1) return "Humidity";
    if (lower.indexOf("6e400002") != -1) return "NUS TX (Write)";
    if (lower.indexOf("6e400003") != -1) return "NUS RX (Notify)";
    if (lower.indexOf("43484152-2dab-3241") != -1) return "RACE TX (Write)";
    if (lower.indexOf("43484152-2dab-3141") != -1 || lower.indexOf("43484152-2dab-3041") != -1) return "RACE RX (Notify)";
    if (lower == "fef1" || lower.indexOf("0000fef1-") != -1) return "Airoha FOTA TX";
    if (lower == "fef2" || lower.indexOf("0000fef2-") != -1) return "Airoha FOTA RX";
    if (lower.indexOf("bfd869fa-a3f2") != -1) return "Sony RACE TX";
    if (lower.indexOf("2a6b6575-faf6") != -1) return "Sony RACE RX";
    if (lower == "ffe1" || lower.indexOf("0000ffe1-") != -1) return "Serial Data";

    if (lower.length() > 8) return "Char " + uuidStr;
    return "Char 0x" + lower;
}

static const char *getFilterModeName(GattFilterMode mode) {
    switch (mode) {
        case FILTER_CONNECTABLE: return "Connectable Only";
        case FILTER_WITH_SERVICES: return "With Advertised Services";
        case FILTER_HID: return "HID / Input (0x1812)";
        case FILTER_UART: return "Serial / NUS / UART";
        case FILTER_AUDIO: return "Audio / Media / AVRCP";
        case FILTER_SENSORS: return "Sensors / Health";
        case FILTER_CUSTOM_128: return "Custom 128-bit UUIDs";
        default: return "All Connectable";
    }
}

//=============================================================================
// Continuous Scanner Callbacks
//=============================================================================

class GattScanCallbacks : public NimBLEScanCallbacks {
public:
    void onDiscovered(const NimBLEAdvertisedDevice *dev) override {}

    void onResult(const NimBLEAdvertisedDevice *dev) override {
        processDevice(dev);
    }

private:
    void processDevice(const NimBLEAdvertisedDevice *dev) {
        if (!dev || !g_gattScanState.active) return;
        if (!g_gattScanState.mutex) return;

        // Try-take mutex with 0 timeout so NimBLE host task is never blocked
        if (xSemaphoreTake(g_gattScanState.mutex, 0) != pdTRUE) return;

        g_gattScanState.totalPackets++;

        int8_t rssi = dev->getRSSI();
        if (rssi == 0) rssi = -100;
        if (rssi < g_gattSettings.minRssi) {
            xSemaphoreGive(g_gattScanState.mutex);
            return;
        }

        uint8_t addrType = dev->getAddressType();
        if (g_gattSettings.addrTypeFilter == 1 && addrType != BLE_ADDR_PUBLIC) {
            xSemaphoreGive(g_gattScanState.mutex);
            return;
        }
        if (g_gattSettings.addrTypeFilter == 2 && addrType == BLE_ADDR_PUBLIC) {
            xSemaphoreGive(g_gattScanState.mutex);
            return;
        }

        bool isConn = dev->isConnectable();

        // Tag inspection
        char tag[8] = "ADV";
        if (isConn) strcpy(tag, "GATT");

        // Parse AD structures from raw payload safely without heap allocations
        const auto &payload = dev->getPayload();
        if (!payload.empty()) {
            const uint8_t *pData = payload.data();
            size_t pLen = payload.size();
            size_t offset = 0;
            while (offset + 1 < pLen) {
                uint8_t fieldLen = pData[offset];
                if (fieldLen == 0 || offset + 1 + fieldLen > pLen) break;
                uint8_t adType = pData[offset + 1];
                const uint8_t *val = &pData[offset + 2];
                uint8_t valLen = fieldLen - 1;

                if (adType == 0x02 || adType == 0x03) { // 16-bit Service UUIDs (Incomplete / Complete)
                    for (size_t k = 0; k + 1 < valLen; k += 2) {
                        uint16_t uuid = (uint16_t)val[k] | ((uint16_t)val[k + 1] << 8);
                        if (uuid == 0x1812 || uuid == 0x1124) { strcpy(tag, "HID"); break; }
                        else if (uuid == 0xFFE0 || uuid == 0xFFF0) { strcpy(tag, "UART"); break; }
                        else if (uuid == 0x110E || uuid == 0x110F || uuid == 0x1843) { strcpy(tag, "AUD"); break; }
                        else if (uuid == 0x180D || uuid == 0x181A || uuid == 0x1809 || uuid == 0x180F) { strcpy(tag, "SENS"); break; }
                        else if (uuid == 0xFE2C) { strcpy(tag, "FP"); break; }
                    }
                } else if (adType == 0x06 || adType == 0x07) { // 128-bit Service UUIDs
                    if (strcmp(tag, "GATT") == 0 || strcmp(tag, "ADV") == 0) {
                        strcpy(tag, "128b");
                    }
                }
                offset += 1 + fieldLen;
            }
        }

        // Filter Mode Matching
        bool match = false;
        switch (g_gattScanState.filterMode) {
            case FILTER_CONNECTABLE:
                match = true;
                break;
            case FILTER_WITH_SERVICES:
                match = (strcmp(tag, "GATT") != 0 && strcmp(tag, "ADV") != 0);
                break;
            case FILTER_HID:
                match = (strcmp(tag, "HID") == 0);
                break;
            case FILTER_UART:
                match = (strcmp(tag, "UART") == 0);
                break;
            case FILTER_AUDIO:
                match = (strcmp(tag, "AUD") == 0);
                break;
            case FILTER_SENSORS:
                match = (strcmp(tag, "SENS") == 0);
                break;
            case FILTER_CUSTOM_128:
                match = (strcmp(tag, "128b") == 0);
                break;
        }

        if (!match) {
            xSemaphoreGive(g_gattScanState.mutex);
            return;
        }

        const uint8_t *devVal = dev->getAddress().getVal();
        uint32_t now = millis();

        // 1. Update existing device if already seen
        for (size_t i = 0; i < g_gattScanState.count; i++) {
            if (memcmp(g_gattScanState.devices[i].macBytes, devVal, 6) == 0) {
                g_gattScanState.devices[i].rssi = rssi;
                g_gattScanState.devices[i].lastSeen = now;
                g_gattScanState.devices[i].packetCount++;
                if (isConn) g_gattScanState.devices[i].isConnectable = true;

                if (g_gattScanState.devices[i].name[0] == '\0' && dev->haveName()) {
                    std::string dName = dev->getName();
                    if (!dName.empty() && dName != "(null)" && dName != "null" && dName != "NULL" && dName != "<no name>") {
                        strncpy(g_gattScanState.devices[i].name, dName.c_str(), sizeof(g_gattScanState.devices[i].name) - 1);
                        g_gattScanState.devices[i].name[sizeof(g_gattScanState.devices[i].name) - 1] = '\0';
                    }
                }

                if (g_gattScanState.devices[i].vendor[0] == '\0') {
                    if (dev->haveManufacturerData()) {
                        std::string mfg = dev->getManufacturerData();
                        if (mfg.length() >= 2) {
                            uint16_t companyId = (uint8_t)mfg[0] | ((uint16_t)(uint8_t)mfg[1] << 8);
                            const char *comp = getBleCompanyIdName(companyId);
                            if (comp) {
                                strncpy(g_gattScanState.devices[i].vendor, comp, sizeof(g_gattScanState.devices[i].vendor) - 1);
                                g_gattScanState.devices[i].vendor[sizeof(g_gattScanState.devices[i].vendor) - 1] = '\0';
                            }
                        }
                    }
                    if (g_gattScanState.devices[i].vendor[0] == '\0' && addrType == BLE_ADDR_PUBLIC) {
                        const char *oui = getBleOuiNameFromMacBytes(devVal);
                        if (oui) {
                            strncpy(g_gattScanState.devices[i].vendor, oui, sizeof(g_gattScanState.devices[i].vendor) - 1);
                            g_gattScanState.devices[i].vendor[sizeof(g_gattScanState.devices[i].vendor) - 1] = '\0';
                        }
                    }
                }

                if (strcmp(g_gattScanState.devices[i].tag, "GATT") == 0 || strcmp(g_gattScanState.devices[i].tag, "ADV") == 0) {
                    if (strcmp(tag, "GATT") != 0 && strcmp(tag, "ADV") != 0) {
                        strncpy(g_gattScanState.devices[i].tag, tag, sizeof(g_gattScanState.devices[i].tag) - 1);
                        g_gattScanState.devices[i].tag[sizeof(g_gattScanState.devices[i].tag) - 1] = '\0';
                    }
                }

                xSemaphoreGive(g_gattScanState.mutex);
                return;
            }
        }

        // 2. New device: append if space available
        if (g_gattScanState.count < GATT_MAX_SCAN_DEVICES) {
            auto &d = g_gattScanState.devices[g_gattScanState.count];
            memcpy(d.macBytes, devVal, 6);
            d.address = dev->getAddress();
            d.addressType = addrType;
            snprintf(d.macStr, sizeof(d.macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                     devVal[5], devVal[4], devVal[3], devVal[2], devVal[1], devVal[0]);

            if (dev->haveName()) {
                std::string dName = dev->getName();
                if (!dName.empty() && dName != "(null)" && dName != "null" && dName != "NULL" && dName != "<no name>") {
                    strncpy(d.name, dName.c_str(), sizeof(d.name) - 1);
                    d.name[sizeof(d.name) - 1] = '\0';
                } else {
                    d.name[0] = '\0';
                }
            } else {
                d.name[0] = '\0';
            }

            d.vendor[0] = '\0';
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
            if (d.vendor[0] == '\0' && addrType == BLE_ADDR_PUBLIC) {
                const char *oui = getBleOuiNameFromMacBytes(devVal);
                if (oui) {
                    strncpy(d.vendor, oui, sizeof(d.vendor) - 1);
                    d.vendor[sizeof(d.vendor) - 1] = '\0';
                }
            }

            d.rssi = rssi;
            d.isConnectable = isConn;
            strncpy(d.tag, tag, sizeof(d.tag) - 1);
            d.tag[sizeof(d.tag) - 1] = '\0';
            d.lastSeen = now;
            d.packetCount = 1;

            g_gattScanState.count++;
        } else {
            // Buffer full: replace device with weakest RSSI if this device is stronger
            size_t minIdx = 0;
            int8_t minRssi = g_gattScanState.devices[0].rssi;
            for (size_t i = 1; i < g_gattScanState.count; i++) {
                if (g_gattScanState.devices[i].rssi < minRssi) {
                    minRssi = g_gattScanState.devices[i].rssi;
                    minIdx = i;
                }
            }
            if (rssi > minRssi) {
                auto &d = g_gattScanState.devices[minIdx];
                memcpy(d.macBytes, devVal, 6);
                d.address = dev->getAddress();
                d.addressType = addrType;
                snprintf(d.macStr, sizeof(d.macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                         devVal[5], devVal[4], devVal[3], devVal[2], devVal[1], devVal[0]);

                if (dev->haveName()) {
                    std::string dName = dev->getName();
                    if (!dName.empty() && dName != "(null)" && dName != "null" && dName != "NULL" && dName != "<no name>") {
                        strncpy(d.name, dName.c_str(), sizeof(d.name) - 1);
                        d.name[sizeof(d.name) - 1] = '\0';
                    } else {
                        d.name[0] = '\0';
                    }
                } else {
                    d.name[0] = '\0';
                }

                d.vendor[0] = '\0';
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
                if (d.vendor[0] == '\0' && addrType == BLE_ADDR_PUBLIC) {
                    const char *oui = getBleOuiNameFromMacBytes(devVal);
                    if (oui) {
                        strncpy(d.vendor, oui, sizeof(d.vendor) - 1);
                        d.vendor[sizeof(d.vendor) - 1] = '\0';
                    }
                }

                d.rssi = rssi;
                d.isConnectable = isConn;
                strncpy(d.tag, tag, sizeof(d.tag) - 1);
                d.tag[sizeof(d.tag) - 1] = '\0';
                d.lastSeen = now;
                d.packetCount = 1;
            }
        }

        xSemaphoreGive(g_gattScanState.mutex);
    }
};

static GattScanCallbacks g_gattScanCallbacks;

struct BleConnDiagInfo {
    int lastErrorCode = 0;
    int lastDisconnectReason = 0;
    uint8_t ownAddrType = BLE_OWN_ADDR_PUBLIC;
    uint8_t peerAddrType = BLE_ADDR_PUBLIC;
    uint8_t phyMask = 1;
    uint16_t itvlMin = 0;
    uint16_t itvlMax = 0;
    uint16_t latency = 0;
    uint16_t timeout = 0;
    uint16_t scanItvl = 0;
    uint16_t scanWin = 0;
    int strategyTried = 0;
    int totalStrategies = 0;
};
static BleConnDiagInfo g_lastConnDiag;

class GattExplorerClientCallbacks : public NimBLEClientCallbacks {
public:
    volatile bool connectDone = false;
    volatile bool connectSuccess = false;
    volatile int connectErrorCode = 0;

    void reset() {
        connectDone = false;
        connectSuccess = false;
        connectErrorCode = 0;
    }

    void onConnect(NimBLEClient *pClient) override {
        connectDone = true;
        connectSuccess = true;
        connectErrorCode = 0;
        NimBLEConnInfo info = pClient->getConnInfo();
        Serial.println(F("\n[BLE-DBG] >>> Connection ESTABLISHED! <<<"));
        Serial.printf("[BLE-DBG] Handle: %d, Peer: %s, MTU: %d\n",
                      pClient->getConnHandle(),
                      pClient->getPeerAddress().toString().c_str(),
                      pClient->getMTU());
        Serial.printf("[BLE-DBG] Interval: %.2f ms, Latency: %d, Supervision Timeout: %d ms\n",
                      info.getConnInterval() * 1.25f,
                      info.getConnLatency(),
                      info.getConnTimeout() * 10);
    }

    void onConnectFail(NimBLEClient *pClient, int reason) override {
        connectDone = true;
        connectSuccess = false;
        connectErrorCode = reason;
        g_lastBleError = reason;
        g_lastConnDiag.lastErrorCode = reason;
        String desc = getBleErrorDescription(reason);
        Serial.printf("\n[BLE-DBG] >>> Connection FAILED! Reason: 0x%02X (%d) -> %s <<<\n",
                      reason, reason, desc.c_str());
        Serial.printf("[BLE-DBG] Target: %s (Type: %s)\n",
                      pClient->getPeerAddress().toString().c_str(),
                      (pClient->getPeerAddress().getType() == BLE_ADDR_PUBLIC) ? "PUBLIC" : "RANDOM");
    }

    void onDisconnect(NimBLEClient *pClient, int reason) override {
        g_lastBleDisconnectReason = reason;
        g_lastConnDiag.lastDisconnectReason = reason;
        if (g_lastBleError == 0) g_lastBleError = reason;
        String desc = getBleErrorDescription(reason);
        Serial.printf("\n[BLE-DBG] >>> Disconnected! Reason: 0x%02X (%d) -> %s <<<\n",
                      reason, reason, desc.c_str());
    }

    bool onConnParamsUpdateRequest(NimBLEClient *pClient, const ble_gap_upd_params *params) override {
        Serial.printf("[BLE-DBG] Remote ConnParamsUpdateRequest: min=%.2fms max=%.2fms lat=%d tmo=%dms -> Accepted\n",
                      params->itvl_min * 1.25f, params->itvl_max * 1.25f, params->latency, params->supervision_timeout * 10);
        return true;
    }
};

static GattExplorerClientCallbacks g_gattClientCallbacks;

//=============================================================================
// Forward Declarations
//=============================================================================

static void runContinuousScan(GattFilterMode filterMode);
static void exploreGattDevice(GattScannedDevice &device);
static void browseServicesAndChars(NimBLEClient *pClient, const GattScannedDevice &device);
static void handleCharacteristicActions(NimBLEClient *pClient, NimBLERemoteCharacteristic *pChar, const String &serviceName);
static void readStandardDeviceInfo(NimBLEClient *pClient);
static bool dumpDeviceGattToStorage(NimBLEClient *pClient, const GattScannedDevice &device, String *outFilePath = nullptr);
static void runAutoDumpAll();

//=============================================================================
// Continuous Scan Implementation (Live Interactive Scanner)
//=============================================================================

static void runContinuousScan(GattFilterMode filterMode) {
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

    g_currentFilter = filterMode;
    g_gattScanState.reset(filterMode);

    pBLEScan->setScanCallbacks(&g_gattScanCallbacks, true);
    pBLEScan->setActiveScan(false); // passive: captures all devices safely and reliably without transmitting
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    pBLEScan->setDuplicateFilter(false);
    pBLEScan->setMaxResults(0);
    pBLEScan->clearResults();

    drawMainBorder(true);

    pBLEScan->start(0, false);
    g_scanActive = true;

    int selectedIdx = 0;
    int scrollOffset = 0;
    uint32_t lastUiUpdate = 0;
    int animFrame = 0;
    const char *spinner = "|/-\\";
    bool needsRedraw = true;
    bool scanPaused = false;
    bool devicePicked = false;
    GattScannedDevice pickedDevice;

    // Snapshot buffer for UI rendering to minimize lock hold time (static to protect task stack)
    static GattScannedDevice uiDevices[GATT_MAX_SCAN_DEVICES];
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
                    g_gattScanState.reset(filterMode);
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
            if (g_gattScanState.mutex && xSemaphoreTake(g_gattScanState.mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                uiCount = g_gattScanState.count;
                uiPackets = g_gattScanState.totalPackets;
                memcpy(uiDevices, g_gattScanState.devices, uiCount * sizeof(GattScannedDevice));
                xSemaphoreGive(g_gattScanState.mutex);
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
            pickedDevice = uiDevices[selectedIdx];
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
            String title = "GATT SCAN ";
            if (filterMode != FILTER_CONNECTABLE) {
                title = "[" + String(getFilterModeName(filterMode)) + "] ";
            }
            title += scanPaused ? "[PAUSED]" : ("[" + String(spinner[animFrame]) + "]");
            tft.drawString(gattFitText(title, tftWidth / 2 + 10), BORDER_PAD_X, headerY);

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

                    String devLabel = "[" + String(uiDevices[itemIdx].tag) + "] ";
                    if (uiDevices[itemIdx].name[0] != '\0') {
                        devLabel += String(uiDevices[itemIdx].name);
                    } else if (uiDevices[itemIdx].vendor[0] != '\0') {
                        devLabel += "[" + String(uiDevices[itemIdx].vendor) + "] " + String(uiDevices[itemIdx].macStr).substring(9);
                    } else {
                        devLabel += String(uiDevices[itemIdx].macStr);
                    }

                    devLabel += " " + String(rssi) + "d";
                    if (uiDevices[itemIdx].addressType == BLE_ADDR_PUBLIC) {
                        devLabel += " [P]";
                    }

                    while (devLabel.length() > 3 && tft.textWidth(devLabel.c_str()) > maxTextW) {
                        devLabel.remove(devLabel.length() - 1);
                    }
                    tft.drawString(devLabel, textX, rowY + 1);
                } else {
                    tft.fillRect(BORDER_PAD_X, rowY, tftWidth - 2 * BORDER_PAD_X, lineH, bruceConfig.bgColor);
                    if (uiCount == 0 && r == 0) {
                        tft.setTextColor(bruceConfig.secColor, bruceConfig.bgColor);
                        tft.drawString("Listening for connectable devices...", BORDER_PAD_X + 4, rowY + 1);
                    }
                }
            }

            // 3. Footer
            tft.fillRect(BORDER_PAD_X, footerY, tftWidth - 2 * BORDER_PAD_X, lineH, bruceConfig.bgColor);
            tft.setTextColor(getColorVariation(bruceConfig.priColor, 8, -1), bruceConfig.bgColor);
            const char *footText = g_gattPickCallback ? "ENTER/SEL pick   ESC back" : "ENTER/SEL explore   ESC back";
            tft.drawCentreString(footText, tftWidth / 2, footerY + 1, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(25));
    }

    // Stop scanner cleanly
    g_gattScanState.stop();
    if (pBLEScan) {
        pBLEScan->stop();
        pBLEScan->clearResults();
        pBLEScan->setScanCallbacks(nullptr);
    }
    g_scanActive = false;

    // Synchronize g_discoveredDevices for Auto-Dump and external callers
    g_discoveredDevices.clear();
    for (size_t i = 0; i < g_gattScanState.count; i++) {
        g_discoveredDevices.push_back(g_gattScanState.devices[i]);
    }

    if (!devicePicked && !bleWasActiveBefore) {
        stopBLEStack();
    }

    // Drain keys
    vTaskDelay(pdMS_TO_TICKS(150));
    check(SelPress);
    check(EscPress);
    _getKeyPress();

    if (devicePicked) {
        if (g_gattPickCallback) {
            g_gattPickCallback(pickedDevice);
        } else {
            exploreGattDevice(pickedDevice);
        }
    }
}

//=============================================================================
// Robust Multi-Strategy GATT Connection
//=============================================================================

bool gattConnectWithStrategies(const NimBLEAddress &target, NimBLEClient **outClient, int *outError, bool *outUserCancelled) {
    if (outError) *outError = 0;
    if (outUserCancelled) *outUserCancelled = false;
    g_lastBleDisconnectReason = 0;
    g_lastBleError = 0;

    memset(&g_lastConnDiag, 0, sizeof(g_lastConnDiag));
    g_lastConnDiag.ownAddrType = BLE_OWN_ADDR_PUBLIC;
    g_lastConnDiag.peerAddrType = target.getType();
    g_lastConnDiag.phyMask = BLE_GAP_LE_PHY_1M_MASK;

    BLEStateManager::initBLE("Bruce-GATT", ESP_PWR_LVL_P9);
    NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
    NimBLEDevice::setSecurityAuth(false, false, false);

    // Prepare target addresses: primary (detected address type) and fallback (flipped address type)
    NimBLEAddress primaryTarget = target;
    uint8_t primaryType = target.getType();
    uint8_t fallbackType = (primaryType == BLE_ADDR_PUBLIC) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
    ble_addr_t flippedAddr = *target.getBase();
    flippedAddr.type = fallbackType;
    NimBLEAddress fallbackTarget(flippedAddr);

    struct StrategyConfig {
        const char *name;
        const NimBLEAddress *targetAddr;
        bool useCustomParams;
        uint16_t itvlMin;
        uint16_t itvlMax;
        uint16_t latency;
        uint16_t timeout;
        uint16_t scanItvl;
        uint16_t scanWin;
    };

    const StrategyConfig strategies[] = {
        // Strategy 1: Primary target address type with native ESP-IDF / NimBLE stack defaults (best controller compatibility)
        {"Primary [Stack Native Defaults]", &primaryTarget, false, 0, 0, 0, 0, 0, 0},
        // Strategy 2: Fallback target address type with native stack defaults
        {"Fallback [Flipped Type Defaults]", &fallbackTarget, false, 0, 0, 0, 0, 0, 0},
        // Strategy 3: Primary target with relaxed smartphone timing (30ms - 50ms)
        {"Primary [Relaxed 30-50ms]", &primaryTarget, true, 24, 40, 0, 500, 32, 16},
        // Strategy 4: Fallback target with relaxed smartphone timing (30ms - 50ms)
        {"Fallback [Flipped Type 30-50ms]", &fallbackTarget, true, 24, 40, 0, 500, 32, 16},
    };

    const size_t totalStrats = sizeof(strategies) / sizeof(strategies[0]);
    g_lastConnDiag.totalStrategies = (int)totalStrats;

    Serial.println(F("\n=================================================="));
    Serial.printf("[BLE-DBG] Starting GATT Connection Handshake Sequence\n");
    Serial.printf("[BLE-DBG] Target MAC: %s (%s)\n", target.toString().c_str(), (primaryType == BLE_ADDR_PUBLIC) ? "PUBLIC" : "RANDOM");
    Serial.printf("[BLE-DBG] Local Own Addr Type: %s (MAC: %s)\n",
                  (NimBLEDevice::getAddress().getType() == BLE_ADDR_PUBLIC) ? "PUBLIC" : "RANDOM",
                  NimBLEDevice::getAddress().toString().c_str());
#if CONFIG_BT_NIMBLE_EXT_ADV
    Serial.printf("[BLE-DBG] PHY Control: Forced LE 1M PHY (0x%02X)\n", BLE_GAP_LE_PHY_1M_MASK);
#endif
    Serial.println(F("=================================================="));

    int lastErr = 0;
    const char *spinnerChars = "|/-\\";
    int spinnerIdx = 0;

    for (size_t i = 0; i < totalStrats; i++) {
        if (check(EscPress) || check(PrevPress)) {
            Serial.println(F("[BLE-DBG] Aborted by user keypress before strategy."));
            if (outUserCancelled) *outUserCancelled = true;
            if (outError) *outError = lastErr;
            return false;
        }

        const auto &strat = strategies[i];
        g_lastConnDiag.strategyTried = (int)i + 1;
        g_lastConnDiag.peerAddrType = strat.targetAddr->getType();
        g_lastConnDiag.itvlMin = strat.itvlMin;
        g_lastConnDiag.itvlMax = strat.itvlMax;
        g_lastConnDiag.latency = strat.latency;
        g_lastConnDiag.timeout = strat.timeout;
        g_lastConnDiag.scanItvl = strat.scanItvl;
        g_lastConnDiag.scanWin = strat.scanWin;

        NimBLEClient *pClient = NimBLEDevice::createClient();
        if (!pClient) {
            lastErr = 0x107;
            g_lastConnDiag.lastErrorCode = 0x107;
            continue;
        }

#if CONFIG_BT_NIMBLE_EXT_ADV
        // Enforce pure 1M PHY connection on ESP32-S3 builds to prevent Extended Multi-PHY rejection
        pClient->setConnectPhy(BLE_GAP_LE_PHY_1M_MASK);
#endif

        NimBLEClient::Config cfg = pClient->getConfig();
        cfg.exchangeMTU = 0; // Decouple immediate ATT MTU exchange
        cfg.connectFailRetries = 1;
        cfg.asyncConnect = 1;
        pClient->setConfig(cfg);

        g_gattClientCallbacks.reset();
        pClient->setClientCallbacks(&g_gattClientCallbacks, false);
        uint32_t perStratTimeoutMs = (g_gattSettings.timeoutSec > 0 ? g_gattSettings.timeoutSec : 5) * 1000;
        pClient->setConnectTimeout(perStratTimeoutMs);

        if (strat.useCustomParams) {
            pClient->setConnectionParams(strat.itvlMin, strat.itvlMax, strat.latency, strat.timeout, strat.scanItvl, strat.scanWin);
        }

        Serial.printf("[BLE-DBG] [Strat %d/%d] %s -> Target: %s (%s)\n",
                      (int)i + 1, (int)totalStrats, strat.name,
                      strat.targetAddr->toString().c_str(),
                      (strat.targetAddr->getType() == BLE_ADDR_PUBLIC) ? "PUBLIC" : "RANDOM");
        if (strat.useCustomParams) {
            Serial.printf("[BLE-DBG]   Params: itvl=%.2f-%.2fms lat=%d tmo=%dms scanItvl=%.2fms scanWin=%.2fms\n",
                          strat.itvlMin * 1.25f, strat.itvlMax * 1.25f, strat.latency, strat.timeout * 10,
                          strat.scanItvl * 0.625f, strat.scanWin * 0.625f);
        } else {
            Serial.printf("[BLE-DBG]   Params: Stack Native Defaults\n");
        }

        // Initiate connection attempt asynchronously
        if (!pClient->connect(*strat.targetAddr, /*deleteAttributes=*/true, /*asyncConnect=*/true, /*exchangeMTU=*/false)) {
            int err = pClient->getLastError();
            if (err != 0) lastErr = err;
            g_lastConnDiag.lastErrorCode = lastErr;
            NimBLEDevice::deleteClient(pClient);
            pClient = nullptr;
            vTaskDelay(50 / portTICK_PERIOD_MS);
            continue;
        }

        // Active non-blocking polling loop with live user abort check and screen updates
        uint32_t startMs = millis();
        uint32_t lastUiTick = 0;
        bool strategySuccess = false;
        bool strategyFailed = false;

        while (millis() - startMs < perStratTimeoutMs) {
            // Check for user cancellation (ESC / Prev)
            if (check(EscPress) || check(PrevPress)) {
                Serial.println(F("[BLE-DBG] Aborted by user keypress during connection attempt."));
                pClient->cancelConnect();
                vTaskDelay(50 / portTICK_PERIOD_MS);
                NimBLEDevice::deleteClient(pClient);
                if (outUserCancelled) *outUserCancelled = true;
                if (outError) *outError = -1;
                return false;
            }

            // Check if connection succeeded
            if (g_gattClientCallbacks.connectDone) {
                if (g_gattClientCallbacks.connectSuccess || pClient->isConnected()) {
                    strategySuccess = true;
                    break;
                } else {
                    strategyFailed = true;
                    lastErr = g_gattClientCallbacks.connectErrorCode;
                    break;
                }
            }

            if (pClient->isConnected()) {
                strategySuccess = true;
                break;
            }

            // Periodic UI updates (spinner + timer + cancel reminder)
            uint32_t now = millis();
            if (now - lastUiTick >= 100) {
                lastUiTick = now;
                spinnerIdx = (spinnerIdx + 1) % 4;
                uint32_t elapsed = now - startMs;
                int remainSec = (elapsed < perStratTimeoutMs) ? (int)((perStratTimeoutMs - elapsed + 999) / 1000) : 0;

                GattUiGeom g = gattUiGeom();
                tft.setTextSize(FP);
                tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);

                int stratY = g.footY - 24;
                int statusY = g.footY - 12;

                // Strategy line
                String stratLine = "Strat " + String((int)i + 1) + "/" + String((int)totalStrats) + ": " + strat.name;
                tft.fillRect(BORDER_PAD_X, stratY, tftWidth - 2 * BORDER_PAD_X, 10 * FP, bruceConfig.bgColor);
                tft.drawString(gattFitText(stratLine, tftWidth - 2 * BORDER_PAD_X), BORDER_PAD_X, stratY);

                // Live status & countdown
                String statusLine = "[" + String(spinnerChars[spinnerIdx]) + "] Handshake... (" + String(remainSec) + "s left)";
                if (i > 0 && lastErr != 0) {
                    char errTag[24];
                    snprintf(errTag, sizeof(errTag), " (prev: 0x%02X)", (unsigned int)lastErr);
                    statusLine += errTag;
                }
                tft.fillRect(BORDER_PAD_X, statusY, tftWidth - 2 * BORDER_PAD_X, 10 * FP, bruceConfig.bgColor);
                tft.drawString(gattFitText(statusLine, tftWidth - 2 * BORDER_PAD_X), BORDER_PAD_X, statusY);

                // Cancel reminder
                tft.fillRect(BORDER_PAD_X, g.footY, tftWidth - 2 * BORDER_PAD_X, 10 * FP, bruceConfig.bgColor);
                tft.setTextColor(gattDimColor(), bruceConfig.bgColor);
                tft.drawCentreString("Press [ESC] to Cancel", tftWidth / 2, g.footY, 1);
            }

            vTaskDelay(20 / portTICK_PERIOD_MS);
        }

        if (strategySuccess) {
            Serial.printf("[BLE-DBG] >>> SUCCESS with Strat %d: %s <<<\n\n", (int)i + 1, strat.name);
            *outClient = pClient;
            return true;
        }

        // Connection failed or timed out on this strategy
        if (!strategyFailed) {
            Serial.printf("[BLE-DBG] [Strat %d/%d] Timeout (%ums)\n", (int)i + 1, (int)totalStrats, (unsigned int)perStratTimeoutMs);
            lastErr = BLE_HS_ETIMEOUT;
            pClient->cancelConnect();
        }

        int err = pClient->getLastError();
        if (err == 0 && g_lastBleDisconnectReason != 0) err = g_lastBleDisconnectReason;
        if (err == 0 && g_lastBleError != 0) err = g_lastBleError;
        if (err != 0) lastErr = err;
        g_lastConnDiag.lastErrorCode = lastErr;

        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;

        // Delay briefly between strategies before next attempt, still checking for ESC
        uint32_t pauseStart = millis();
        while (millis() - pauseStart < 100) {
            if (check(EscPress) || check(PrevPress)) {
                Serial.println(F("[BLE-DBG] Aborted by user keypress during pause."));
                if (outUserCancelled) *outUserCancelled = true;
                if (outError) *outError = lastErr;
                return false;
            }
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
    }

    if (outError) *outError = lastErr;
    Serial.printf("[BLE-DBG] Connection sequence FAILED. Final Code: 0x%02X (%d)\n\n", lastErr, lastErr);
    return false;
}

//=============================================================================
// On-Screen Connection Diagnostics Display
//=============================================================================

static void showBleConnectDiagnostics(const BleConnDiagInfo &diag, const GattScannedDevice &device) {
    drawMainBorderWithTitle("BLE CONNECT DIAG");
    tft.setTextSize(FP);

    int y = gattUiGeom().top;
    const int step = 11;

    // Status Code & Error description
    tft.setTextColor(TFT_RED, bruceConfig.bgColor);
    String codeStr = "Code: 0x" + String(diag.lastErrorCode, HEX) + " (" + String(diag.lastErrorCode) + ")";
    tft.drawString(codeStr, BORDER_PAD_X, y); y += step;
    tft.drawString(gattFitText(getBleErrorDescription(diag.lastErrorCode), tftWidth - 2 * BORDER_PAD_X), BORDER_PAD_X, y); y += step + 2;

    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.drawString("Target: " + gattFitText(device.name, tftWidth - 2 * BORDER_PAD_X), BORDER_PAD_X, y); y += step;
    tft.drawString("MAC:    " + String(device.address.toString().c_str()), BORDER_PAD_X, y); y += step;

    String typeLine = "Peer: " + String((device.addressType == BLE_ADDR_PUBLIC) ? "PUB" : "RND") +
                      "  Own: " + String((diag.ownAddrType == BLE_ADDR_PUBLIC) ? "PUB" : "RND");
    tft.drawString(typeLine, BORDER_PAD_X, y); y += step;

#if CONFIG_BT_NIMBLE_EXT_ADV
    String phyLine = "PHY: LE 1M (Forced S3)  RSSI: " + String(device.rssi) + "dBm";
#else
    String phyLine = "PHY: LE 1M (Legacy)  RSSI: " + String(device.rssi) + "dBm";
#endif
    tft.drawString(phyLine, BORDER_PAD_X, y); y += step;

    String paramLine = "Int: " + String(diag.itvlMin * 5 / 4) + "-" + String(diag.itvlMax * 5 / 4) + "ms Tmo:" + String(diag.timeout * 10) + "ms";
    tft.drawString(paramLine, BORDER_PAD_X, y); y += step;

    String stratLine = "Strat tried: " + String(diag.strategyTried) + "/" + String(diag.totalStrategies) + " [Failed]";
    tft.drawString(stratLine, BORDER_PAD_X, y); y += step + 3;

    tft.setTextColor(bruceConfig.secColor, bruceConfig.bgColor);
    tft.fillRect(BORDER_PAD_X, gattUiGeom().footY, tftWidth - 2 * BORDER_PAD_X, 10 * FP, bruceConfig.bgColor);
    tft.drawCentreString("Press SEL or ESC to return", tftWidth / 2, gattUiGeom().footY, 1);

    while (true) {
        if (check(EscPress) || check(SelPress) || check(PrevPress) || check(NextPress)) {
            break;
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

//=============================================================================
// GATT Device Exploration & Connection
//=============================================================================

static void exploreGattDevice(GattScannedDevice &device) {
    if (device.vendor[0] == '\0' && device.addressType == BLE_ADDR_PUBLIC) {
        String v = resolveBleOui(device.address, true);
        if (v.length() > 0) {
            strncpy(device.vendor, v.c_str(), sizeof(device.vendor) - 1);
            device.vendor[sizeof(device.vendor) - 1] = '\0';
        }
    }

    drawMainBorderWithTitle("GATT CONNECT");
    tft.setTextSize(FP);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);

    GattUiGeom g = gattUiGeom();
    int rowY = g.top;
    const int step = 11;
    tft.drawString("Target: " + gattFitText(device.name[0] != '\0' ? String(device.name) : String(device.macStr), tftWidth - 20), BORDER_PAD_X, rowY); rowY += step;
    if (device.vendor[0] != '\0') {
        tft.drawString("Vendor: " + gattFitText(device.vendor, tftWidth - 20), BORDER_PAD_X, rowY); rowY += step;
    }
    tft.drawString("MAC:    " + String(device.address.toString().c_str()), BORDER_PAD_X, rowY); rowY += step;
    tft.drawString("Type:   " + String((device.addressType == BLE_ADDR_PUBLIC) ? "PUBLIC" : "RANDOM"), BORDER_PAD_X, rowY); rowY += step;
    tft.drawString("Signal: " + String(device.rssi) + " dBm", BORDER_PAD_X, rowY);

    NimBLEClient *pClient = nullptr;
    int connError = 0;
    bool userCancelled = false;
    bool connected = gattConnectWithStrategies(device.address, &pClient, &connError, &userCancelled);

    if (userCancelled) {
        displayWarning("Connection cancelled", true);
        return;
    }

    if (!connected || !pClient) {
        showBleConnectDiagnostics(g_lastConnDiag, device);
        return;
    }

    int statusY = g.footY - 12;
    tft.fillRect(BORDER_PAD_X, statusY, tftWidth - 2 * BORDER_PAD_X, 10 * FP, bruceConfig.bgColor);
    tft.setTextColor(TFT_GREEN, bruceConfig.bgColor);
    tft.drawString(gattFitText("Connected! Discovering attributes...", tftWidth - 2 * BORDER_PAD_X), BORDER_PAD_X, statusY);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    pClient->discoverAttributes();

    const auto &services = pClient->getServices(true);
    if (services.empty()) {
        displayWarning("No GATT services exposed", true);
        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient);
        return;
    }

    displaySuccess("Found " + String((int)services.size()) + " services!");
    delay(300);

    // Device Operation Menu
    int devCursor = 0;
    while (pClient->isConnected()) {
        std::vector<GattMenuItem> devOps;

        devOps.push_back({"1. Browse Services & Chars", [pClient, &device]() {
            browseServicesAndChars(pClient, device);
        }});

        devOps.push_back({"2. Read Device Info & Batt", [pClient]() {
            readStandardDeviceInfo(pClient);
        }});

        devOps.push_back({"3. RACE Assessment (Airoha)", [pClient, &device]() {
            launchRaceForDevice(pClient, device.name[0] != '\0' ? String(device.name) : String(device.macStr), device.address);
        }});

        devOps.push_back({"4. Dump GATT Tree to Storage", [pClient, &device]() {
            String path;
            if (dumpDeviceGattToStorage(pClient, device, &path)) {
                displaySuccess("Saved: " + path, true);
            } else {
                displayError("Dump failed", true);
            }
        }});

        // Runs the live tracker while keeping the active GATT connection open for high-rate
        // link-layer RSSI reads; returns right back to this menu on ESC.
        devOps.push_back({"5. Track this device", [pClient, &device]() {
            String label = device.name[0] != '\0' ? String(device.name) : String(device.macStr);
            bleTrackerRun(String(device.macStr), label, pClient);
        }});

        devOps.push_back({"6. Disconnect & Back", [pClient]() {
            if (pClient->isConnected()) pClient->disconnect();
        }});

        const char *title = device.name[0] != '\0' ? device.name : device.macStr;
        int sel = gattMenu(title, devOps, "SEL choose  ESC back", &devCursor);
        if (sel == -1 || sel == (int)devOps.size() - 1 || !pClient->isConnected()) {
            break;
        }
    }

    if (pClient->isConnected()) {
        pClient->disconnect();
    }
    NimBLEDevice::deleteClient(pClient);
    delay(50);
}

//=============================================================================
// Hierarchical Services & Characteristics Browser (Compact / FP Font)
//=============================================================================

static void browseServicesAndChars(NimBLEClient *pClient, const GattScannedDevice &device) {
    if (!pClient || !pClient->isConnected()) {
        displayError("Device disconnected", true);
        return;
    }

    int serviceCursor = 0;

    while (pClient->isConnected()) {
        const auto &services = pClient->getServices(true);
        if (services.empty()) {
            displayWarning("No services found", true);
            return;
        }

        int srvCount = (int)services.size();
        int totalCount = srvCount + 1; // services + back

        auto srvDrawer = [&services, srvCount](int idx, int x, int y, int w, bool sel) {
            uint16_t fg = sel ? bruceConfig.bgColor : bruceConfig.priColor;
            uint16_t bg = sel ? bruceConfig.priColor : bruceConfig.bgColor;
            tft.setTextColor(fg, bg);
            tft.setTextSize(FP);

            if (idx < srvCount) {
                NimBLERemoteService *srv = services[idx];
                String uStr = String(srv->getUUID().toString().c_str());
                String sName = getGattServiceName(uStr);
                String label = "[" + String(idx + 1) + "] " + sName;
                tft.drawString(gattFitText(label, w), x, y, 1);
            } else {
                tft.drawString("< Back to Device Menu", x, y, 1);
            }
        };

        int chosenSrv = gattListLoop("GATT SERVICES", totalCount, "SEL view chars  ESC back", srvDrawer, &serviceCursor);
        if (chosenSrv < 0 || chosenSrv == srvCount) {
            break;
        }

        NimBLERemoteService *selectedSrv = services[chosenSrv];
        String sUStr = String(selectedSrv->getUUID().toString().c_str());
        String sName = getGattServiceName(sUStr);

        int charCursor = 0;

        // Browse characteristics of selected service
        while (pClient->isConnected()) {
            const auto &chars = selectedSrv->getCharacteristics(true);
            if (chars.empty()) {
                displayWarning("No characteristics in service", true);
                break;
            }

            int charCount = (int)chars.size();
            int charTotalCount = charCount + 1;

            auto charDrawer = [&chars, charCount](int idx, int x, int y, int w, bool sel) {
                uint16_t fg = sel ? bruceConfig.bgColor : bruceConfig.priColor;
                uint16_t bg = sel ? bruceConfig.priColor : bruceConfig.bgColor;
                tft.setTextColor(fg, bg);
                tft.setTextSize(FP);

                if (idx < charCount) {
                    NimBLERemoteCharacteristic *ch = chars[idx];
                    String cuStr = String(ch->getUUID().toString().c_str());
                    String cName = getGattCharName(cuStr);

                    String flags = "";
                    if (ch->canRead()) flags += "R";
                    if (ch->canWrite() || ch->canWriteNoResponse()) flags += "W";
                    if (ch->canNotify()) flags += "N";
                    if (ch->canIndicate()) flags += "I";

                    String cLabel = cName;
                    if (flags.length() > 0) cLabel += " [" + flags + "]";
                    tft.drawString(gattFitText(cLabel, w), x, y, 1);
                } else {
                    tft.drawString("< Back to Services", x, y, 1);
                }
            };

            int chosenChar = gattListLoop(sName.c_str(), charTotalCount, "SEL actions  ESC back", charDrawer, &charCursor);
            if (chosenChar < 0 || chosenChar == charCount) {
                break;
            }

            handleCharacteristicActions(pClient, chars[chosenChar], sName);
        }
    }
}

//=============================================================================
// Characteristic Actions (Read / Write / Live Notify / Details)
//=============================================================================

static void handleCharacteristicActions(NimBLEClient *pClient, NimBLERemoteCharacteristic *pChar, const String &serviceName) {
    if (!pChar || !pClient) return;

    String cuStr = String(pChar->getUUID().toString().c_str());
    String cName = getGattCharName(cuStr);
    int actCursor = 0;

    while (pClient->isConnected()) {
        std::vector<GattMenuItem> actOptions;

        if (pChar->canRead()) {
            actOptions.push_back({"1. Read Value", [pChar, cName, cuStr]() {
                NimBLEAttValue val = pChar->readValue();

                std::vector<String> lines;
                lines.push_back("Name: " + cName);
                lines.push_back("UUID: " + cuStr);
                lines.push_back("Size: " + String(val.size()) + " bytes");
                lines.push_back("");

                if (val.size() == 0) {
                    lines.push_back("[Value is empty / 0 bytes]");
                } else {
                    lines.push_back("--- HEX ---");
                    String hexStr = "";
                    for (size_t i = 0; i < val.size(); i++) {
                        if (val[i] < 0x10) hexStr += "0";
                        hexStr += String(val[i], HEX) + " ";
                        if ((i + 1) % 8 == 0 && i < val.size() - 1) {
                            lines.push_back(hexStr);
                            hexStr = "";
                        }
                    }
                    if (hexStr.length() > 0) lines.push_back(hexStr);

                    lines.push_back("");
                    lines.push_back("--- ASCII ---");
                    String asciiStr = "";
                    for (size_t i = 0; i < val.size(); i++) {
                        char c = (char)val[i];
                        asciiStr += (c >= 32 && c <= 126) ? c : '.';
                    }
                    lines.push_back(asciiStr);
                }

                gattShowScrollableReport("CHAR READ", lines);
            }});
        }

        if (pChar->canWrite() || pChar->canWriteNoResponse()) {
            actOptions.push_back({"2. Write String", [pChar]() {
                String input = keyboard("", 64, "Input String to Write:");
                if (input != "\x1B" && input.length() > 0) {
                    bool ok = pChar->writeValue((const uint8_t *)input.c_str(), input.length(), pChar->canWrite());
                    if (ok) displaySuccess("Written: " + String(input.length()) + " B", true);
                    else displayError("Write failed", true);
                }
            }});

            actOptions.push_back({"3. Write Hex Bytes", [pChar]() {
                String hexIn = hex_keyboard("", 64, "Input Hex Bytes (e.g. 0102FF):");
                if (hexIn != "\x1B" && hexIn.length() > 0) {
                    hexIn.replace(" ", "");
                    hexIn.replace(":", "");
                    if (hexIn.length() % 2 != 0) hexIn = "0" + hexIn;

                    std::vector<uint8_t> bytes;
                    for (size_t i = 0; i < hexIn.length(); i += 2) {
                        String byteHex = hexIn.substring(i, i + 2);
                        uint8_t b = (uint8_t)strtol(byteHex.c_str(), nullptr, 16);
                        bytes.push_back(b);
                    }

                    if (!bytes.empty()) {
                        bool ok = pChar->writeValue(bytes.data(), bytes.size(), pChar->canWrite());
                        if (ok) displaySuccess("Written " + String((int)bytes.size()) + " hex bytes", true);
                        else displayError("Hex write failed", true);
                    }
                }
            }});
        }

        if (pChar->canNotify() || pChar->canIndicate()) {
            actOptions.push_back({"4. Live Notify Stream", [pClient, pChar, cName]() {
                GattUiGeom sg = gattUiGeom();
                drawMainBorderWithTitle("LIVE STREAM");
                tft.setTextSize(FP);
                tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);

                tft.drawString("Stream: " + gattFitText(cName, tftWidth - 20), BORDER_PAD_X, sg.top);
                tft.drawString("Press [SEL] or [ESC] to stop", BORDER_PAD_X, sg.top + 11);
                tft.drawFastHLine(BORDER_PAD_X, sg.top + 23, tftWidth - 2 * BORDER_PAD_X, bruceConfig.priColor);

                static StaticSemaphore_t s_streamMutexBuf;
                static SemaphoreHandle_t s_streamMutex = nullptr;
                if (!s_streamMutex) {
                    s_streamMutex = xSemaphoreCreateMutexStatic(&s_streamMutexBuf);
                }

                static int s_notifyCount = 0;
                static char s_lastNotifyBuf[96] = {0};
                static bool s_hasNewNotify = false;

                if (s_streamMutex && xSemaphoreTake(s_streamMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    s_notifyCount = 0;
                    s_lastNotifyBuf[0] = '\0';
                    s_hasNewNotify = false;
                    xSemaphoreGive(s_streamMutex);
                }

                bool subOk = pChar->subscribe(true, [](NimBLERemoteCharacteristic *chr, uint8_t *pData, size_t length, bool isNotify) {
                    if (!s_streamMutex || xSemaphoreTake(s_streamMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
                    s_notifyCount++;
                    char hexBuf[48] = {0};
                    size_t hexLen = 0;
                    size_t maxBytes = std::min(length, (size_t)10);
                    for (size_t i = 0; i < maxBytes; i++) {
                        int written = snprintf(hexBuf + hexLen, sizeof(hexBuf) - hexLen, "%02X ", pData[i]);
                        if (written > 0) hexLen += (size_t)written;
                    }
                    if (length > maxBytes && hexLen + 3 < sizeof(hexBuf)) {
                        strcat(hexBuf, "..");
                    }
                    snprintf(s_lastNotifyBuf, sizeof(s_lastNotifyBuf), "#%d [%uB]: %s", s_notifyCount, (unsigned int)length, hexBuf);
                    s_hasNewNotify = true;
                    xSemaphoreGive(s_streamMutex);
                });

                if (!subOk) {
                    displayError("Subscription failed", true);
                    return;
                }

                int lineY = sg.top + 27;
                while (true) {
                    if (check(EscPress) || check(SelPress)) break;

                    if (!pClient || !pClient->isConnected()) {
                        displayWarning("Device disconnected", true);
                        break;
                    }

                    char drawBuf[96] = {0};
                    bool hasNew = false;
                    if (s_streamMutex && xSemaphoreTake(s_streamMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        if (s_hasNewNotify) {
                            strncpy(drawBuf, s_lastNotifyBuf, sizeof(drawBuf) - 1);
                            drawBuf[sizeof(drawBuf) - 1] = '\0';
                            s_hasNewNotify = false;
                            hasNew = true;
                        }
                        xSemaphoreGive(s_streamMutex);
                    }

                    if (hasNew) {
                        tft.fillRect(BORDER_PAD_X, lineY, tftWidth - 2 * BORDER_PAD_X, 10 * FP, bruceConfig.bgColor);
                        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
                        tft.drawString(gattFitText(String(drawBuf), tftWidth - 2 * BORDER_PAD_X), BORDER_PAD_X, lineY);

                        lineY += 12;
                        if (lineY > tftHeight - 20) {
                            lineY = sg.top + 27;
                            tft.fillRect(BORDER_PAD_X, lineY, tftWidth - 2 * BORDER_PAD_X, tftHeight - lineY - 18, bruceConfig.bgColor);
                        }
                    }
                    vTaskDelay(30 / portTICK_PERIOD_MS);
                }

                if (pClient && pClient->isConnected()) {
                    pChar->unsubscribe();
                    displaySuccess("Unsubscribed", true);
                }
            }});
        }

        actOptions.push_back({"5. Characteristic Details", [pChar, cName, cuStr]() {
            std::vector<String> lines;
            lines.push_back("Name:     " + cName);
            lines.push_back("UUID:     " + cuStr);
            lines.push_back("Handle:   0x" + String(pChar->getHandle(), HEX));
            lines.push_back("Read:     " + String(pChar->canRead() ? "YES" : "NO"));
            lines.push_back("Write:    " + String(pChar->canWrite() ? "YES" : "NO"));
            lines.push_back("WriteNR:  " + String(pChar->canWriteNoResponse() ? "YES" : "NO"));
            lines.push_back("Notify:   " + String(pChar->canNotify() ? "YES" : "NO"));
            lines.push_back("Indicate: " + String(pChar->canIndicate() ? "YES" : "NO"));

            gattShowScrollableReport("CHAR DETAILS", lines);
        }});

        actOptions.push_back({"< Back to Characteristics", []() {}});

        int actSel = gattMenu(cName.c_str(), actOptions, "SEL choose  ESC back", &actCursor);
        if (actSel == -1 || actSel == (int)actOptions.size() - 1 || !pClient->isConnected()) {
            break;
        }
    }
}

//=============================================================================
// Read Standard Device Information (DIS 0x180A) & Battery (0x180F)
//=============================================================================

static void readStandardDeviceInfo(NimBLEClient *pClient) {
    if (!pClient || !pClient->isConnected()) {
        displayError("Not connected", true);
        return;
    }

    drawMainBorderWithTitle("DEVICE REPORT");
    tft.setTextSize(FP);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.drawString("Querying Device Information...", BORDER_PAD_X, gattUiGeom().top);

    String manufacturer = "N/A";
    String model = "N/A";
    String serial = "N/A";
    String firmware = "N/A";
    String hardware = "N/A";
    String battery = "N/A";

    NimBLERemoteService *pDis = pClient->getService(NimBLEUUID((uint16_t)0x180A));
    if (pDis) {
        NimBLERemoteCharacteristic *cMfr = pDis->getCharacteristic(NimBLEUUID((uint16_t)0x2A29));
        if (cMfr && cMfr->canRead()) manufacturer = String(cMfr->readValue().c_str());

        NimBLERemoteCharacteristic *cMod = pDis->getCharacteristic(NimBLEUUID((uint16_t)0x2A24));
        if (cMod && cMod->canRead()) model = String(cMod->readValue().c_str());

        NimBLERemoteCharacteristic *cSer = pDis->getCharacteristic(NimBLEUUID((uint16_t)0x2A25));
        if (cSer && cSer->canRead()) serial = String(cSer->readValue().c_str());

        NimBLERemoteCharacteristic *cFw = pDis->getCharacteristic(NimBLEUUID((uint16_t)0x2A26));
        if (cFw && cFw->canRead()) firmware = String(cFw->readValue().c_str());

        NimBLERemoteCharacteristic *cHw = pDis->getCharacteristic(NimBLEUUID((uint16_t)0x2A27));
        if (cHw && cHw->canRead()) hardware = String(cHw->readValue().c_str());
    }

    NimBLERemoteService *pBatt = pClient->getService(NimBLEUUID((uint16_t)0x180F));
    if (pBatt) {
        NimBLERemoteCharacteristic *cBatt = pBatt->getCharacteristic(NimBLEUUID((uint16_t)0x2A19));
        if (cBatt && cBatt->canRead()) {
            NimBLEAttValue v = cBatt->readValue();
            if (v.size() > 0) battery = String((int)v[0]) + "%";
        }
    }

    std::vector<String> lines;
    String resolvedVendor = resolveBleOui(pClient->getPeerAddress(), true);
    if (resolvedVendor.length() > 0 && manufacturer == "N/A") {
        manufacturer = resolvedVendor + " (OUI)";
    }
    lines.push_back("Manufacturer: " + manufacturer);
    if (resolvedVendor.length() > 0 && manufacturer != "N/A" && manufacturer.indexOf(resolvedVendor) == -1) {
        lines.push_back("OUI Vendor:   " + resolvedVendor);
    }
    lines.push_back("Model Number: " + model);
    lines.push_back("Serial Num:   " + serial);
    lines.push_back("Firmware Rev: " + firmware);
    lines.push_back("Hardware Rev: " + hardware);
    lines.push_back("Battery Lvl:  " + battery);

    gattShowScrollableReport("DEVICE REPORT", lines);
}

//=============================================================================
// Dump GATT Tree to SD / LittleFS
//=============================================================================

static bool dumpDeviceGattToStorage(NimBLEClient *pClient, const GattScannedDevice &device, String *outFilePath) {
    if (!pClient || !pClient->isConnected()) return false;

    FS *fs;
    String storageType = "";
    if (getFsStorage(fs) && fs == &SD) {
        storageType = "SD";
    } else if (setupLittleFS()) {
        fs = &LittleFS;
        storageType = "LittleFS";
    }

    if (!fs || storageType.isEmpty()) return false;

    if (!fs->exists("/ble_dumps")) fs->mkdir("/ble_dumps");

    String cleanMac = String(device.address.toString().c_str());
    cleanMac.replace(":", "");
    String filepath = "/ble_dumps/GATT_" + cleanMac + ".txt";

    File file = fs->open(filepath, FILE_WRITE);
    if (!file) return false;

    file.println("==================================================");
    file.println("BRUCE GATT EXPLORER DUMP");
    file.println("==================================================");
    file.println("Device Name: " + String(device.name));
    file.println("Address:     " + String(device.address.toString().c_str()));
    file.println("Addr Type:   " + String((device.addressType == BLE_ADDR_PUBLIC) ? "PUBLIC" : "RANDOM"));
    file.println("RSSI:        " + String(device.rssi) + " dBm");
    file.println("Dump Time:   " + String(millis()) + " ms");
    file.println("==================================================\n");

    const auto &services = pClient->getServices(true);
    for (size_t s = 0; s < services.size(); s++) {
        NimBLERemoteService *srv = services[s];
        String sUStr = String(srv->getUUID().toString().c_str());
        String sName = getGattServiceName(sUStr);

        file.printf("[%d] SERVICE: %s (UUID: %s)\n", (int)(s + 1), sName.c_str(), sUStr.c_str());

        const auto &chars = srv->getCharacteristics(true);
        for (size_t c = 0; c < chars.size(); c++) {
            NimBLERemoteCharacteristic *ch = chars[c];
            String cuStr = String(ch->getUUID().toString().c_str());
            String cName = getGattCharName(cuStr);

            String flags = "";
            if (ch->canRead()) flags += "READ ";
            if (ch->canWrite()) flags += "WRITE ";
            if (ch->canWriteNoResponse()) flags += "WRITE_NR ";
            if (ch->canNotify()) flags += "NOTIFY ";
            if (ch->canIndicate()) flags += "INDICATE ";
            flags.trim();

            file.printf("    - CHAR: %s (UUID: %s) [0x%04X] [%s]\n", cName.c_str(), cuStr.c_str(), ch->getHandle(), flags.c_str());

            if (ch->canRead()) {
                NimBLEAttValue val = ch->readValue();
                if (val.size() > 0) {
                    String hexStr = "";
                    String ascStr = "";
                    for (size_t b = 0; b < val.size(); b++) {
                        if (val[b] < 0x10) hexStr += "0";
                        hexStr += String(val[b], HEX) + " ";
                        char chChar = (char)val[b];
                        ascStr += (chChar >= 32 && chChar <= 126) ? chChar : '.';
                    }
                    file.printf("        HEX: %s\n", hexStr.c_str());
                    file.printf("        ASC: \"%s\"\n", ascStr.c_str());
                } else {
                    file.println("        VAL: [0 bytes]");
                }
            }
        }
        file.println("");
    }

    file.println("==================================================");
    file.println("END OF DUMP");
    file.println("==================================================");
    file.close();

    if (outFilePath) *outFilePath = storageType + ":" + filepath;
    return true;
}

//=============================================================================
// Auto-Dump All Discovered Devices
//=============================================================================

static void runAutoDumpAll() {
    if (g_discoveredDevices.empty()) {
        displayWarning("No devices discovered to dump", true);
        return;
    }

    GattUiGeom sg = gattUiGeom();
    drawMainBorderWithTitle("AUTO-DUMP ALL");
    tft.setTextSize(FP);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);

    int total = g_discoveredDevices.size();
    int successCount = 0;
    int lineY = sg.top;

    for (int i = 0; i < total; i++) {
        if (check(EscPress)) {
            tft.drawString("Aborted by user.", BORDER_PAD_X, lineY);
            break;
        }

        const auto &dev = g_discoveredDevices[i];
        String devName = dev.name[0] != '\0' ? String(dev.name) : String(dev.macStr);
        String progress = "[" + String(i + 1) + "/" + String(total) + "] " + devName;
        tft.drawString(gattFitText(progress, tftWidth - 20), BORDER_PAD_X, lineY);
        lineY += 12;

        NimBLEClient *pClient = nullptr;
        bool userCancelled = false;
        bool connected = gattConnectWithStrategies(dev.address, &pClient, nullptr, &userCancelled);
        if (userCancelled) {
            tft.drawString("Aborted by user.", BORDER_PAD_X, lineY);
            break;
        }
        if (connected && pClient) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            pClient->discoverAttributes();
            String path;
            if (dumpDeviceGattToStorage(pClient, dev, &path)) {
                successCount++;
            }
            pClient->disconnect();
            NimBLEDevice::deleteClient(pClient);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);

        if (lineY > tftHeight - 24) {
            lineY = sg.top;
            tft.fillRect(BORDER_PAD_X, lineY, tftWidth - 2 * BORDER_PAD_X, tftHeight - lineY - 14, bruceConfig.bgColor);
        }
    }

    std::vector<String> lines;
    lines.push_back("Auto-Dump Finished!");
    lines.push_back("Success: " + String(successCount) + " / " + String(total) + " devices");
    lines.push_back("Files saved in: /ble_dumps/");

    gattShowScrollableReport("AUTO-DUMP SUMMARY", lines);
}

//=============================================================================
// Settings Menu (Compact / FP Font)
//=============================================================================

void gattSettingsMenu() {
    int cursor = 0;
    while (true) {
        std::vector<GattMenuItem> setOptions;

        String rssiLabel = "1. Min RSSI: " + ((g_gattSettings.minRssi <= -100) ? String("None (-100dBm)") : String(g_gattSettings.minRssi) + " dBm");
        setOptions.push_back({rssiLabel, []() {
            if (g_gattSettings.minRssi <= -100) g_gattSettings.minRssi = -85;
            else if (g_gattSettings.minRssi == -85) g_gattSettings.minRssi = -75;
            else if (g_gattSettings.minRssi == -75) g_gattSettings.minRssi = -65;
            else g_gattSettings.minRssi = -100;
        }});

        String toLabel = "2. Timeout: " + String(g_gattSettings.timeoutSec) + " sec";
        setOptions.push_back({toLabel, []() {
            if (g_gattSettings.timeoutSec == 3) g_gattSettings.timeoutSec = 5;
            else if (g_gattSettings.timeoutSec == 5) g_gattSettings.timeoutSec = 8;
            else if (g_gattSettings.timeoutSec == 8) g_gattSettings.timeoutSec = 12;
            else if (g_gattSettings.timeoutSec == 12) g_gattSettings.timeoutSec = 15;
            else g_gattSettings.timeoutSec = 3;
        }});

        String respToLabel = "3. Resp Timeout: " + ((g_gattSettings.scanRespTimeout == 0) ? String("Off (0ms)") : String(g_gattSettings.scanRespTimeout) + " ms");
        setOptions.push_back({respToLabel, []() {
            if (g_gattSettings.scanRespTimeout == 0) g_gattSettings.scanRespTimeout = 50;
            else if (g_gattSettings.scanRespTimeout == 50) g_gattSettings.scanRespTimeout = 100;
            else if (g_gattSettings.scanRespTimeout == 100) g_gattSettings.scanRespTimeout = 250;
            else if (g_gattSettings.scanRespTimeout == 250) g_gattSettings.scanRespTimeout = 500;
            else g_gattSettings.scanRespTimeout = 0;
        }});

        String discLabel = "4. Inc onDiscovered: " + String(g_gattSettings.includeOnDiscovered ? "Yes" : "No");
        setOptions.push_back({discLabel, []() {
            g_gattSettings.includeOnDiscovered = !g_gattSettings.includeOnDiscovered;
        }});

        String addrLabel = "5. Addr Type: ";
        if (g_gattSettings.addrTypeFilter == 0) addrLabel += "Any";
        else if (g_gattSettings.addrTypeFilter == 1) addrLabel += "Public Only";
        else addrLabel += "Random Only";
        setOptions.push_back({addrLabel, []() {
            g_gattSettings.addrTypeFilter = (g_gattSettings.addrTypeFilter + 1) % 3;
        }});

        String maxLabel = "6. Max Devices (RSSI): " + String(g_gattSettings.maxDevices) + " dev";
        setOptions.push_back({maxLabel, []() {
            if (g_gattSettings.maxDevices == 20) g_gattSettings.maxDevices = 40;
            else if (g_gattSettings.maxDevices == 40) g_gattSettings.maxDevices = 60;
            else if (g_gattSettings.maxDevices == 60) g_gattSettings.maxDevices = 80;
            else g_gattSettings.maxDevices = 20;

            if (g_discoveredDevices.size() > (size_t)g_gattSettings.maxDevices) {
                std::sort(g_discoveredDevices.begin(), g_discoveredDevices.end(), [](const GattScannedDevice &a, const GattScannedDevice &b) {
                    return a.rssi > b.rssi;
                });
                g_discoveredDevices.resize(g_gattSettings.maxDevices);
            }
        }});

        setOptions.push_back({"< Back to GATT Menu", []() {}});

        int sel = gattMenu("GATT SETTINGS", setOptions, "SEL toggle  ESC back", &cursor);
        if (sel == -1 || sel == (int)setOptions.size() - 1) {
            break;
        }
    }
}

//=============================================================================
// Reusable Scan-And-Pick Entry Point (for other features, e.g. BLE Tracker)
//=============================================================================

bool gattScanAndPick(String &outName, String &outMac, int &outRssi, uint8_t &outAddrType) {
    GattScannedDevice picked;
    bool hasPick = false;
    g_gattPickCallback = [&picked, &hasPick](const GattScannedDevice &device) {
        picked = device;
        hasPick = true;
    };
    runContinuousScan(g_currentFilter);
    g_gattPickCallback = nullptr;

    if (hasPick) {
        outName = (picked.name[0] != '\0') ? String(picked.name) : String(picked.macStr);
        outMac = String(picked.macStr);
        outRssi = picked.rssi;
        outAddrType = picked.addressType;
        return true;
    }
    return false;
}

//=============================================================================
// Main GATT Explorer Menu (Compact / FP Font)
//=============================================================================

void gattExplorerMenu() {
    int cursor = 0;
    while (true) {
        std::vector<GattMenuItem> menuOps;

        menuOps.push_back({"1. Scan Connectable (Live)", [=]() {
            runContinuousScan(FILTER_CONNECTABLE);
        }});

        menuOps.push_back({"2. Filter by Service", [=]() {
            int fltCursor = 0;
            while (true) {
                std::vector<GattMenuItem> fltOps;
                fltOps.push_back({"1. All Connectable", [=]() { runContinuousScan(FILTER_CONNECTABLE); }});
                fltOps.push_back({"2. With Advertised Services", [=]() { runContinuousScan(FILTER_WITH_SERVICES); }});
                fltOps.push_back({"3. HID / Input (0x1812)", [=]() { runContinuousScan(FILTER_HID); }});
                fltOps.push_back({"4. Serial / NUS / UART", [=]() { runContinuousScan(FILTER_UART); }});
                fltOps.push_back({"5. Audio / Media (0x110E)", [=]() { runContinuousScan(FILTER_AUDIO); }});
                fltOps.push_back({"6. Sensors / Health (0x180D)", [=]() { runContinuousScan(FILTER_SENSORS); }});
                fltOps.push_back({"7. Custom 128-bit UUIDs", [=]() { runContinuousScan(FILTER_CUSTOM_128); }});
                fltOps.push_back({"< Back", []() {}});

                int sel = gattMenu("SERVICE FILTERS", fltOps, "SEL choose  ESC back", &fltCursor);
                if (sel == -1 || sel == (int)fltOps.size() - 1) {
                    break;
                }
            }
        }});

        menuOps.push_back({"3. Auto-Dump to Storage", [=]() {
            runAutoDumpAll();
        }});

        menuOps.push_back({"4. Settings", [=]() {
            gattSettingsMenu();
        }});

        menuOps.push_back({"5. GATT Test Server", [=]() {
            gattServerMenu();
        }});

        menuOps.push_back({"< Back to Bluetooth", []() {}});

        int sel = gattMenu("GATT EXPLORER", menuOps, "SEL choose  ESC back", &cursor);
        if (sel == -1 || sel == (int)menuOps.size() - 1) {
            break;
        }
    }
}

//=============================================================================
// CLI Automation Functions
//=============================================================================

bool gattConnectCli(const String &macStr, uint8_t addrType) {
    NimBLEAddress target(std::string(macStr.c_str()), addrType);
    Serial.printf("[BLE-CLI] Initiating connection to %s (%s)...\n",
                  target.toString().c_str(),
                  (addrType == BLE_ADDR_PUBLIC) ? "PUBLIC" : "RANDOM");

    NimBLEClient *pClient = nullptr;
    int connError = 0;
    bool connected = gattConnectWithStrategies(target, &pClient, &connError);

    if (!connected || !pClient) {
        Serial.printf("[BLE-CLI] Connection FAILED. Code: 0x%02X (%d) -> %s\n",
                      connError, connError, getBleErrorDescription(connError).c_str());
        return false;
    }

    Serial.println(F("[BLE-CLI] Connected successfully! Discovering services..."));
    if (pClient->discoverAttributes()) {
        const auto &services = pClient->getServices(true);
        Serial.printf("[BLE-CLI] Found %d services:\n", (int)services.size());
        for (auto *srv : services) {
            Serial.printf("[BLE-CLI]  + Service: %s\n", srv->getUUID().toString().c_str());
            const auto &chars = srv->getCharacteristics(true);
            for (auto *chr : chars) {
                String props = "";
                if (chr->canRead()) props += "R ";
                if (chr->canWrite()) props += "W ";
                if (chr->canNotify()) props += "N ";
                if (chr->canIndicate()) props += "I ";
                Serial.printf("[BLE-CLI]     - Char: %s [%s]\n", chr->getUUID().toString().c_str(), props.c_str());
            }
        }
    } else {
        Serial.println(F("[BLE-CLI] discoverAttributes failed or incomplete."));
    }

    Serial.println(F("[BLE-CLI] Disconnecting..."));
    pClient->disconnect();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    NimBLEDevice::deleteClient(pClient);
    Serial.println(F("[BLE-CLI] Done."));
    return true;
}

void gattScanCli(int timeoutSec) {
    if (timeoutSec <= 0) timeoutSec = 5;
    Serial.printf("[BLE-CLI] Starting %d-second scan for connectable GATT devices...\n", timeoutSec);

    BLEStateManager::initBLE("Bruce-GATT-Scan", ESP_PWR_LVL_P9);
    NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);

    NimBLEScan *pScan = NimBLEDevice::getScan();
    pScan->clearResults();
    pScan->setActiveScan(true);
    pScan->setInterval(96);
    pScan->setWindow(48);

    NimBLEScanResults results = pScan->getResults(timeoutSec * 1000, false);
    Serial.printf("[BLE-CLI] Scan finished. Found %d devices:\n", (int)results.getCount());

    for (int i = 0; i < results.getCount(); i++) {
        const auto *dev = results.getDevice(i);
        if (!dev) continue;
        String vendor = resolveBleVendor(dev, true);
        Serial.printf("[BLE-CLI] %2d. %s [%s] RSSI:%d dBm Name:\"%s\" Vendor:\"%s\" Conn:%s\n",
                      i + 1,
                      dev->getAddress().toString().c_str(),
                      (dev->getAddress().getType() == BLE_ADDR_PUBLIC) ? "PUB" : "RND",
                      dev->getRSSI(),
                      dev->getName().c_str(),
                      vendor.length() > 0 ? vendor.c_str() : "Unknown",
                      dev->isConnectable() ? "YES" : "NO");
    }

    pScan->stop();
    pScan->clearResults();
}

#endif // !LITE_VERSION
