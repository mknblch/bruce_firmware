#if !defined(LITE_VERSION)

#include "ble_oui.h"
#include "core/sd_functions.h"
#include <FS.h>
#include <LittleFS.h>
#include <SD.h>
#include <globals.h>
#include <algorithm>

//=============================================================================
// Layer 1: Embedded PROGMEM Tables (Zero dynamic RAM overhead)
//=============================================================================

struct BleCompanyIdEntry {
    uint16_t id;
    const char *name;
};

// Bluetooth SIG Assigned Numbers (Manufacturer Specific Data AD Type 0xFF)
static const BleCompanyIdEntry BLE_SIG_COMPANIES[] PROGMEM = {
    {0x0001, "Nokia"},
    {0x0006, "Microsoft"},
    {0x000A, "Qualcomm/CSR"},
    {0x000D, "Texas Instruments"},
    {0x000F, "Broadcom"},
    {0x001D, "Qualcomm"},
    {0x0025, "Kingston"},
    {0x002D, "Sony"},
    {0x0046, "MediaTek"},
    {0x004C, "Apple"},
    {0x0057, "Epson"},
    {0x0059, "Nordic Semi"},
    {0x0060, "Toshiba"},
    {0x006B, "Polar"},
    {0x0075, "Samsung"},
    {0x0087, "Garmin"},
    {0x009B, "Realtek"},
    {0x009E, "Bose"},
    {0x00B5, "Casio"},
    {0x00D2, "Dialog Semi"},
    {0x00E0, "Google"},
    {0x0113, "Harman/JBL"},
    {0x012D, "Sennheiser"},
    {0x0131, "Cypress/Infineon"},
    {0x0147, "Motorola"},
    {0x0157, "Huami/Amazfit"},
    {0x0171, "Amazon"},
    {0x0180, "Beats"},
    {0x01D3, "LG Electronics"},
    {0x0211, "Philips"},
    {0x0224, "Jabra/GN"},
    {0x027D, "Huawei"},
    {0x02AC, "GoPro"},
    {0x02E5, "Espressif"},
    {0x038F, "Xiaomi"},
    {0x0399, "NXP Semi"},
    {0x03E0, "DJI"},
    {0x0499, "Ruuvi"},
    {0x04C0, "Fitbit"},
    {0x052B, "SteelSeries"},
    {0x056A, "Tile"},
    {0x05A7, "Sonos"},
    {0x0594, "Nothing"},
    {0x07D7, "Tuya Smart"},
    {0x0822, "Realme"},
    {0x08A9, "OnePlus"},
    {0x09C5, "Anker"},
    {0x0A5C, "Flipper"},
    {0x0CB6, "Zebra"},
};
static const size_t BLE_SIG_COMPANIES_COUNT = sizeof(BLE_SIG_COMPANIES) / sizeof(BLE_SIG_COMPANIES[0]);

// Common IoT, Peripheral, Board & Appliance Public MAC OUIs (First 3 bytes)
struct BleOuiEntry {
    uint32_t oui; // 24-bit packed: (byte0 << 16) | (byte1 << 8) | byte2
    const char *name;
};

static const BleOuiEntry COMMON_BLE_OUIS[] PROGMEM = {
    // Espressif Systems
    {0xAC6784, "Espressif"},
    {0x246F28, "Espressif"},
    {0x30AEA4, "Espressif"},
    {0x70039F, "Espressif"},
    {0x840D8E, "Espressif"},
    {0x94B97E, "Espressif"},
    {0x240AC4, "Espressif"},
    {0x48E729, "Espressif"},
    {0x4022D8, "Espressif"},
    {0x500291, "Espressif"},
    {0xDC5475, "Espressif"},
    {0x600194, "Espressif"},
    {0x782184, "Espressif"},
    {0x98CDAC, "Espressif"},
    {0x348518, "Espressif"},
    {0x30C6F7, "Espressif"},
    {0xD8BCEF, "Espressif"},

    // Raspberry Pi
    {0xB827EB, "Raspberry Pi"},
    {0xDCA632, "Raspberry Pi"},
    {0xE45F01, "Raspberry Pi"},
    {0x28CDC1, "Raspberry Pi"},
    {0x2CFC67, "Raspberry Pi"},

    // Nordic Semiconductor
    {0xF4CE36, "Nordic Semi"},
    {0xC8FD19, "Nordic Semi"},
    {0xE017D7, "Nordic Semi"},
    {0xD48A3B, "Nordic Semi"},

    // Logitech
    {0x001F20, "Logitech"},
    {0x000420, "Logitech"},
    {0x001BDC, "Logitech"},
    {0x883B5F, "Logitech"},
    {0x6C4008, "Logitech"},
    {0x104FA8, "Logitech"},

    // Apple Inc.
    {0x001A2B, "Apple"},
    {0x001E52, "Apple"},
    {0x002500, "Apple"},
    {0x0017F2, "Apple"},
    {0x040C56, "Apple"},
    {0x087402, "Apple"},
    {0x18AF61, "Apple"},
    {0x38CA84, "Apple"},
    {0x406C8F, "Apple"},
    {0x5855CA, "Apple"},
    {0x685B35, "Apple"},
    {0x701124, "Apple"},
    {0x7CF005, "Apple"},
    {0x88665A, "Apple"},
    {0xAC87A3, "Apple"},
    {0xBC5436, "Apple"},
    {0xDC2B61, "Apple"},
    {0xF099BF, "Apple"},
    {0xF8E903, "Apple"},

    // Samsung Electronics
    {0x0023E7, "Samsung"},
    {0x002637, "Samsung"},
    {0x50F5DA, "Samsung"},
    {0x5492BE, "Samsung"},
    {0x78471D, "Samsung"},
    {0x842519, "Samsung"},
    {0x94652D, "Samsung"},
    {0xBC7EFD, "Samsung"},
    {0xD059E4, "Samsung"},
    {0xF434F0, "Samsung"},
    {0x001599, "Samsung"},
    {0x001247, "Samsung"},

    // Google
    {0x001A11, "Google"},
    {0x3C5AB4, "Google"},
    {0x546009, "Google"},
    {0x94EB2C, "Google"},
    {0xF40343, "Google"},
    {0xF4F5D8, "Google"},
    {0x20DFB9, "Google"},

    // Xiaomi
    {0x546C0E, "Xiaomi"},
    {0x7802F8, "Xiaomi"},
    {0xACF7F3, "Xiaomi"},
    {0x14F65A, "Xiaomi"},
    {0x640980, "Xiaomi"},
    {0x50EC50, "Xiaomi"},

    // Huawei
    {0x001E10, "Huawei"},
    {0x00259E, "Huawei"},
    {0x00464B, "Huawei"},
    {0x200889, "Huawei"},
    {0x7054F5, "Huawei"},
    {0xAC853D, "Huawei"},
    {0xBC25E0, "Huawei"},
    {0xE01954, "Huawei"},

    // Texas Instruments
    {0x0017E9, "Texas Instruments"},
    {0x508CB1, "Texas Instruments"},
    {0x544A16, "Texas Instruments"},
    {0x8030DC, "Texas Instruments"},
    {0x883314, "Texas Instruments"},
    {0x98072D, "Texas Instruments"},
    {0xB09122, "Texas Instruments"},
    {0xCC78AB, "Texas Instruments"},
    {0x00124B, "Texas Instruments"},

    // Tuya Smart IoT
    {0x582D34, "Tuya Smart"},
    {0x68572D, "Tuya Smart"},
    {0x708976, "Tuya Smart"},
    {0x840F2A, "Tuya Smart"},
    {0x9C956E, "Tuya Smart"},
    {0xD81F12, "Tuya Smart"},

    // Sony
    {0x00014A, "Sony"},
    {0x0013E0, "Sony"},
    {0x001D28, "Sony"},
    {0x0024BE, "Sony"},
    {0x280DDC, "Sony"},
    {0x702605, "Sony"},
    {0xFC0F4B, "Sony"},

    // Amazon
    {0x34D270, "Amazon"},
    {0x38F73D, "Amazon"},
    {0x44650D, "Amazon"},
    {0x50DC79, "Amazon"},
    {0x68545A, "Amazon"},
    {0x747548, "Amazon"},
    {0xAC63BE, "Amazon"},
    {0xF0272D, "Amazon"},

    // Bose
    {0x000C8A, "Bose"},
    {0x0452C7, "Bose"},
    {0x08DF1F, "Bose"},
    {0x40ED98, "Bose"},
    {0x806CF9, "Bose"},

    // Garmin
    {0x001C66, "Garmin"},
    {0x00223A, "Garmin"},
    {0x14C14E, "Garmin"},
    {0x583C25, "Garmin"},
    {0x64A2F9, "Garmin"},
    {0x882A5E, "Garmin"},

    // Intel
    {0x001302, "Intel"},
    {0x001500, "Intel"},
    {0x001B21, "Intel"},
    {0x3413E8, "Intel"},
    {0x6805CA, "Intel"},
    {0x8086F2, "Intel"},

    // Realtek
    {0x00072F, "Realtek"},
    {0x00E04C, "Realtek"},
    {0x54AF97, "Realtek"},
    {0x74EE2A, "Realtek"},

    // Broadcom
    {0x0005B5, "Broadcom"},
    {0x001018, "Broadcom"},
    {0x00904C, "Broadcom"},
    {0x2047DA, "Broadcom"},

    // VMware / Virtual
    {0x000569, "VMware"},
    {0x000C29, "VMware"},
    {0x001C14, "VMware"},
    {0x005056, "VMware"},

    // STMicro / Flipper
    {0x0080E1, "STMicro/Flipper"},

    // Philips
    {0x00013E, "Philips"},
    {0x001788, "Philips"},
    {0x30D16B, "Philips"},

    // Zebra
    {0x00074D, "Zebra"},
    {0x00A0F8, "Zebra"},
    {0x84248D, "Zebra"},
};
static const size_t COMMON_BLE_OUIS_COUNT = sizeof(COMMON_BLE_OUIS) / sizeof(COMMON_BLE_OUIS[0]);

//=============================================================================
// Small LRU Cache (8 entries, ~256 bytes RAM) to minimize SD Card seeks
//=============================================================================

struct OuiCacheEntry {
    uint32_t oui;
    char name[28];
};

static OuiCacheEntry s_ouiCache[8];
static uint8_t s_ouiCacheIndex = 0;

static String checkOuiCache(uint32_t oui) {
    for (int i = 0; i < 8; i++) {
        if (s_ouiCache[i].oui == oui && s_ouiCache[i].name[0] != '\0') {
            return String(s_ouiCache[i].name);
        }
    }
    return "";
}

static void addToOuiCache(uint32_t oui, const String &name) {
    if (name.length() == 0) return;
    s_ouiCache[s_ouiCacheIndex].oui = oui;
    strncpy(s_ouiCache[s_ouiCacheIndex].name, name.c_str(), sizeof(s_ouiCache[s_ouiCacheIndex].name) - 1);
    s_ouiCache[s_ouiCacheIndex].name[sizeof(s_ouiCache[s_ouiCacheIndex].name) - 1] = '\0';
    s_ouiCacheIndex = (s_ouiCacheIndex + 1) % 8;
}

//=============================================================================
// Layer 2: SD Card / LittleFS Database Lookup Engine
// Supports both Binary search (oui.bin) and Text scan (oui.txt)
//=============================================================================

static const char *const OUI_BIN_PATHS[] = {
    "/ble/oui.bin",
    "/oui.bin",
    "/ChimeraBLE/oui.bin",
    "/sdcard/oui.bin",
};
static const size_t OUI_BIN_PATHS_COUNT = sizeof(OUI_BIN_PATHS) / sizeof(OUI_BIN_PATHS[0]);

static const char *const OUI_TXT_PATHS[] = {
    "/ble/oui.txt",
    "/oui.txt",
    "/sdcard/oui.txt",
};
static const size_t OUI_TXT_PATHS_COUNT = sizeof(OUI_TXT_PATHS) / sizeof(OUI_TXT_PATHS[0]);

static String lookupBinaryOuiFile(File &file, uint32_t targetOui) {
    size_t fileSize = file.size();
    if (fileSize < 7) return "";

    // Auto-detect fixed record size (standard 24, 32, 16, or 7 bytes)
    int recordSize = 24;
    if (fileSize % 32 == 0) recordSize = 32;
    else if (fileSize % 24 == 0) recordSize = 24;
    else if (fileSize % 16 == 0) recordSize = 16;
    else if (fileSize % 7 == 0) recordSize = 7;

    int low = 0;
    int high = (int)(fileSize / recordSize) - 1;

    while (low <= high) {
        int mid = low + (high - low) / 2;
        if (!file.seek((uint32_t)mid * recordSize)) break;

        uint8_t buf[3];
        if (file.read(buf, 3) != 3) break;

        uint32_t curOui = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];

        if (curOui == targetOui) {
            if (recordSize == 7) {
                // Format: [3 bytes OUI][4 bytes string offset]
                uint32_t strOff = 0;
                if (file.read((uint8_t *)&strOff, 4) == 4 && file.seek(strOff)) {
                    char strBuf[32];
                    size_t n = file.readBytesUntil('\0', strBuf, sizeof(strBuf) - 1);
                    strBuf[n] = '\0';
                    String res = String(strBuf);
                    res.trim();
                    return res;
                }
            } else {
                // Fixed-size record with inline string
                char strBuf[36];
                int toRead = std::min((int)sizeof(strBuf) - 1, recordSize - 3);
                int n = file.read((uint8_t *)strBuf, toRead);
                if (n > 0) {
                    strBuf[n] = '\0';
                    String res = String(strBuf);
                    res.trim();
                    return res;
                }
            }
            break;
        } else if (curOui < targetOui) {
            low = mid + 1;
        } else {
            high = mid - 1;
        }
    }

    return "";
}

static String lookupTextOuiFile(File &file, uint32_t targetOui) {
    char targetHex[7];
    snprintf(targetHex, sizeof(targetHex), "%06X", (unsigned int)targetOui);

    char line[96];
    while (file.available()) {
        size_t len = file.readBytesUntil('\n', line, sizeof(line) - 1);
        line[len] = '\0';
        if (len < 6) continue;

        // Skip leading whitespace / colons / dashes
        char normHex[7];
        int normIdx = 0;
        int i = 0;
        while (line[i] != '\0' && normIdx < 6) {
            char c = line[i];
            if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f')) {
                normHex[normIdx++] = (c >= 'a' && c <= 'f') ? (c - 'a' + 'A') : c;
            } else if (c != ':' && c != '-' && c != ' ' && c != '\t') {
                break;
            }
            i++;
        }
        normHex[6] = '\0';

        if (normIdx == 6 && strncmp(normHex, targetHex, 6) == 0) {
            while (line[i] == ' ' || line[i] == '\t' || line[i] == '-' || line[i] == ':') i++;
            String res = String(&line[i]);
            res.trim();
            return res;
        }
    }
    return "";
}

static String lookupOuiOnStorage(uint32_t targetOui) {
    // 1. Check SD card if available
    bool hasSd = sdcardMounted;
    if (!hasSd) {
        hasSd = setupSdCard(2);
    }

    if (hasSd) {
        // Try binary database files first ($O(\log N)$ fast binary search)
        for (size_t i = 0; i < OUI_BIN_PATHS_COUNT; i++) {
            if (SD.exists(OUI_BIN_PATHS[i])) {
                File f = SD.open(OUI_BIN_PATHS[i], FILE_READ);
                if (f) {
                    String res = lookupBinaryOuiFile(f, targetOui);
                    f.close();
                    if (res.length() > 0) return res;
                }
            }
        }

        // Try text database files next
        for (size_t i = 0; i < OUI_TXT_PATHS_COUNT; i++) {
            if (SD.exists(OUI_TXT_PATHS[i])) {
                File f = SD.open(OUI_TXT_PATHS[i], FILE_READ);
                if (f) {
                    String res = lookupTextOuiFile(f, targetOui);
                    f.close();
                    if (res.length() > 0) return res;
                }
            }
        }
    }

    // 2. Check LittleFS
    for (size_t i = 0; i < OUI_BIN_PATHS_COUNT; i++) {
        if (LittleFS.exists(OUI_BIN_PATHS[i])) {
            File f = LittleFS.open(OUI_BIN_PATHS[i], FILE_READ);
            if (f) {
                String res = lookupBinaryOuiFile(f, targetOui);
                f.close();
                if (res.length() > 0) return res;
            }
        }
    }

    for (size_t i = 0; i < OUI_TXT_PATHS_COUNT; i++) {
        if (LittleFS.exists(OUI_TXT_PATHS[i])) {
            File f = LittleFS.open(OUI_TXT_PATHS[i], FILE_READ);
            if (f) {
                String res = lookupTextOuiFile(f, targetOui);
                f.close();
                if (res.length() > 0) return res;
            }
        }
    }

    return "";
}

//=============================================================================
// Public API Functions
//=============================================================================

String resolveBleCompanyId(uint16_t companyId) {
    for (size_t i = 0; i < BLE_SIG_COMPANIES_COUNT; i++) {
        if (BLE_SIG_COMPANIES[i].id == companyId) {
            return String(BLE_SIG_COMPANIES[i].name);
        }
    }
    return "";
}

String resolveBleOui(uint32_t oui24, bool allowSdLookup) {
    if (oui24 == 0) return "";

    // 1. Check RAM Cache
    String cached = checkOuiCache(oui24);
    if (cached.length() > 0) {
        return cached;
    }

    // 2. Check Flash Table
    for (size_t i = 0; i < COMMON_BLE_OUIS_COUNT; i++) {
        if (COMMON_BLE_OUIS[i].oui == oui24) {
            String name = String(COMMON_BLE_OUIS[i].name);
            addToOuiCache(oui24, name);
            return name;
        }
    }

    // 3. Fallback to SD Storage
    if (allowSdLookup) {
        String sdRes = lookupOuiOnStorage(oui24);
        if (sdRes.length() > 0) {
            addToOuiCache(oui24, sdRes);
            return sdRes;
        }
    }

    return "";
}

String resolveBleOui(const NimBLEAddress &address, bool allowSdLookup) {
    if (address.getType() != BLE_ADDR_PUBLIC) return "";
    String mac = String(address.toString().c_str());
    if (mac.length() < 8) return "";
    uint32_t oui = 0;
    for (int i = 0; i < 8; i++) {
        char c = mac.charAt(i);
        if (c == ':') continue;
        uint32_t nibble = 0;
        if (c >= '0' && c <= '9') nibble = c - '0';
        else if (c >= 'A' && c <= 'F') nibble = c - 'A' + 10;
        else if (c >= 'a' && c <= 'f') nibble = c - 'a' + 10;
        oui = (oui << 4) | nibble;
    }
    return resolveBleOui(oui, allowSdLookup);
}

String resolveBleVendor(const NimBLEAdvertisedDevice *dev, bool allowSdLookup) {
    if (!dev) return "";

    // 1. Check Bluetooth SIG Company ID from Manufacturer Data (works on Public & RPA addresses)
    if (dev->haveManufacturerData()) {
        std::string mfg = dev->getManufacturerData();
        if (mfg.length() >= 2) {
            uint16_t companyId = (uint8_t)mfg[0] | ((uint16_t)(uint8_t)mfg[1] << 8);
            String comp = resolveBleCompanyId(companyId);
            if (comp.length() > 0) return comp;
        }
    }

    // 2. Check Public MAC Address OUI
    if (dev->getAddressType() == BLE_ADDR_PUBLIC) {
        String ouiName = resolveBleOui(dev->getAddress(), allowSdLookup);
        if (ouiName.length() > 0) return ouiName;
    }

    return "";
}

bool isSdOuiDatabaseAvailable() {
    bool hasSd = sdcardMounted;
    if (!hasSd) hasSd = setupSdCard(2);

    if (hasSd) {
        for (size_t i = 0; i < OUI_BIN_PATHS_COUNT; i++) {
            if (SD.exists(OUI_BIN_PATHS[i])) return true;
        }
        for (size_t i = 0; i < OUI_TXT_PATHS_COUNT; i++) {
            if (SD.exists(OUI_TXT_PATHS[i])) return true;
        }
    }

    for (size_t i = 0; i < OUI_BIN_PATHS_COUNT; i++) {
        if (LittleFS.exists(OUI_BIN_PATHS[i])) return true;
    }
    for (size_t i = 0; i < OUI_TXT_PATHS_COUNT; i++) {
        if (LittleFS.exists(OUI_TXT_PATHS[i])) return true;
    }

    return false;
}

#endif // !LITE_VERSION
