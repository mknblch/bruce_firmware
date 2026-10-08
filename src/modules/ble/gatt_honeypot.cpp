#if !defined(LITE_VERSION)

#include "gatt_honeypot.h"
#include "BLE_Suite.h"
#include "gatt_explorer.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/sd_functions.h"
#include "core/scrollableTextArea.h"
#include "core/utils.h"
#include "modules/ble/ble_tracker.h"
#include <NimBLEDevice.h>
#include <globals.h>
#include <esp_mac.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <SD.h>
#include <vector>
#include <deque>
#include <memory>
#include <algorithm>

//=============================================================================
// Data Structures
//=============================================================================

struct HoneypotCharacteristicDef {
    String uuid;
    uint32_t properties = 0;
    std::vector<uint8_t> readValue;
    String rawValueStr;
    bool isHex = false;
    NimBLECharacteristic *pNimChar = nullptr;
};

struct HoneypotServiceDef {
    String uuid;
    std::vector<HoneypotCharacteristicDef> characteristics;
};

struct HoneypotDeviceDef {
    String name;
    String mac;
    bool isRandomMac = false;
    std::vector<HoneypotServiceDef> services;
};

struct HoneypotState {
    volatile bool isRunning = false;
    volatile bool isConnected = false;
    char peerAddress[20] = "None";
    uint16_t peerConnHandle = 0xFFFF;
    uint16_t peerMtu = 23;
    uint32_t connCount = 0;
    uint32_t readCount = 0;
    uint32_t writeCount = 0;
    uint32_t subCount = 0;
    char lastWritePayload[64] = "";
    std::deque<String> logLines;
    uint32_t logGeneration = 0;
    StaticSemaphore_t logMutexBuf;
    SemaphoreHandle_t logMutex = nullptr;

    HoneypotDeviceDef activeDevice;

    void initMutex() {
        if (!logMutex) {
            logMutex = xSemaphoreCreateMutexStatic(&logMutexBuf);
        }
    }

    void addLog(const String &msg) {
        initMutex();
        if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            logLines.push_back(msg);
            while (logLines.size() > 40) {
                logLines.pop_front();
            }
            logGeneration++;
            xSemaphoreGive(logMutex);
        }
    }

    void setPeer(const char *addr, uint16_t mtu, uint16_t connHandle = 0xFFFF) {
        initMutex();
        if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (addr) {
                strncpy(peerAddress, addr, sizeof(peerAddress) - 1);
                peerAddress[sizeof(peerAddress) - 1] = '\0';
            } else {
                strcpy(peerAddress, "None");
            }
            peerMtu = mtu;
            peerConnHandle = connHandle;
            xSemaphoreGive(logMutex);
        }
    }

    void getPeer(char *outAddr, size_t maxLen, uint16_t *outMtu = nullptr, uint16_t *outConnHandle = nullptr) {
        initMutex();
        if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (outAddr && maxLen > 0) {
                strncpy(outAddr, peerAddress, maxLen - 1);
                outAddr[maxLen - 1] = '\0';
            }
            if (outMtu) *outMtu = peerMtu;
            if (outConnHandle) *outConnHandle = peerConnHandle;
            xSemaphoreGive(logMutex);
        }
    }

    void setLastWrite(const char *val) {
        initMutex();
        if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (val) {
                strncpy(lastWritePayload, val, sizeof(lastWritePayload) - 1);
                lastWritePayload[sizeof(lastWritePayload) - 1] = '\0';
            } else {
                lastWritePayload[0] = '\0';
            }
            xSemaphoreGive(logMutex);
        }
    }

    void reset() {
        isRunning = false;
        isConnected = false;
        connCount = 0;
        readCount = 0;
        writeCount = 0;
        subCount = 0;
        initMutex();
        if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            strcpy(peerAddress, "None");
            peerMtu = 23;
            peerConnHandle = 0xFFFF;
            lastWritePayload[0] = '\0';
            logLines.clear();
            logGeneration = 0;
            xSemaphoreGive(logMutex);
        }
    }
};

static HoneypotState g_hpState;
static uint8_t g_originalBluetoothMac[6] = {};
static bool g_originalBluetoothMacSaved = false;

//=============================================================================
// Helper Functions: Hex & MAC parsing
//=============================================================================

static std::vector<uint8_t> parseHexStringToBytes(const String &hexStr) {
    std::vector<uint8_t> bytes;
    String cleanHex = hexStr;
    cleanHex.replace(" ", "");
    cleanHex.replace("0x", "");
    cleanHex.replace("0X", "");
    cleanHex.replace("hex:", "");
    cleanHex.replace("HEX:", "");
    cleanHex.replace(":", "");
    cleanHex.replace("-", "");

    for (size_t i = 0; i + 1 < cleanHex.length(); i += 2) {
        String bytePart = cleanHex.substring(i, i + 2);
        uint8_t b = (uint8_t)strtoul(bytePart.c_str(), nullptr, 16);
        bytes.push_back(b);
    }
    return bytes;
}

static bool parseMacBytes(const String &macStr, uint8_t mac[6]) {
    if (macStr.isEmpty()) return false;
    std::vector<uint8_t> b = parseHexStringToBytes(macStr);
    if (b.size() == 6) {
        for (int i = 0; i < 6; i++) {
            mac[i] = b[i];
        }
        return true;
    }
    return false;
}

static String bytesToHexString(const uint8_t *data, size_t len, size_t maxBytes = 16) {
    if (!data || len == 0) return "";
    String hex = "";
    size_t count = (len > maxBytes) ? maxBytes : len;
    for (size_t i = 0; i < count; i++) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X ", data[i]);
        hex += buf;
    }
    if (len > maxBytes) hex += "..";
    hex.trim();
    return hex;
}

static String shortUuidStr(const String &uuidStr) {
    if (uuidStr.length() > 8 && uuidStr.startsWith("0000") && uuidStr.indexOf("-0000-1000-8000-00805f9b34fb") != -1) {
        return uuidStr.substring(4, 8);
    }
    if (uuidStr.length() > 8) {
        return uuidStr.substring(0, 8) + "..";
    }
    return uuidStr;
}

//=============================================================================
// JSON Parsing & Built-in Presets
//=============================================================================

static uint32_t parseProperties(JsonVariant propVar) {
    uint32_t props = 0;
    if (propVar.is<JsonArray>()) {
        for (JsonVariant v : propVar.as<JsonArray>()) {
            String p = v.as<String>();
            p.toLowerCase();
            p.trim();
            if (p == "read") props |= NIMBLE_PROPERTY::READ;
            else if (p == "write") props |= NIMBLE_PROPERTY::WRITE;
            else if (p == "write_nr" || p == "write_no_response" || p == "writenr") props |= NIMBLE_PROPERTY::WRITE_NR;
            else if (p == "notify") props |= NIMBLE_PROPERTY::NOTIFY;
            else if (p == "indicate") props |= NIMBLE_PROPERTY::INDICATE;
            else if (p == "broadcast") props |= NIMBLE_PROPERTY::BROADCAST;
            else if (p == "read_enc" || p == "read_auth") props |= NIMBLE_PROPERTY::READ_ENC;
            else if (p == "write_enc" || p == "write_auth") props |= NIMBLE_PROPERTY::WRITE_ENC;
        }
    } else if (propVar.is<const char *>()) {
        String pStr = propVar.as<String>();
        pStr.toLowerCase();
        if (pStr.indexOf("read") != -1) props |= NIMBLE_PROPERTY::READ;
        if (pStr.indexOf("write_nr") != -1 || pStr.indexOf("writenr") != -1) props |= NIMBLE_PROPERTY::WRITE_NR;
        else if (pStr.indexOf("write") != -1) props |= NIMBLE_PROPERTY::WRITE;
        if (pStr.indexOf("notify") != -1) props |= NIMBLE_PROPERTY::NOTIFY;
        if (pStr.indexOf("indicate") != -1) props |= NIMBLE_PROPERTY::INDICATE;
        if (pStr.indexOf("broadcast") != -1) props |= NIMBLE_PROPERTY::BROADCAST;
    } else if (propVar.is<uint32_t>()) {
        props = propVar.as<uint32_t>();
    }
    return props;
}

static String normalizeHoneypotUuid(const String &rawUuid) {
    String uuid = rawUuid;
    uuid.trim();
    if (uuid.startsWith("0x") || uuid.startsWith("0X")) {
        uuid.remove(0, 2);
    }
    return uuid;
}

static bool isValidHoneypotUuid(const String &uuid) {
    NimBLEUUID parsedUuid(std::string(uuid.c_str()));
    return parsedUuid.bitSize() != 0;
}

static bool isNimbleManagedServiceUuid(const String &uuid) {
    NimBLEUUID parsedUuid(std::string(uuid.c_str()));
    return parsedUuid == NimBLEUUID(static_cast<uint16_t>(0x1800)) ||
           parsedUuid == NimBLEUUID(static_cast<uint16_t>(0x1801));
}

static bool parseHoneypotJson(const String &jsonContent, HoneypotDeviceDef &outDevice) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, jsonContent);
    if (err) {
        Serial.printf("[HONEYPOT-JSON] Parse error: %s\n", err.c_str());
        return false;
    }

    JsonObject root;
    if (doc.is<JsonObject>()) {
        root = doc.as<JsonObject>();
        if (root["presets"].is<JsonArray>() && root["presets"].as<JsonArray>().size() > 0) {
            root = root["presets"][0].as<JsonObject>();
        }
    } else if (doc.is<JsonArray>() && doc.as<JsonArray>().size() > 0) {
        root = doc[0].as<JsonObject>();
    } else {
        return false;
    }

    outDevice.services.clear();
    outDevice.name = root["name"].as<String>();
    outDevice.mac = root["mac"].as<String>();
    if (outDevice.name.isEmpty() || outDevice.mac.isEmpty()) return false;

    String macType = root["mac_type"].as<String>();
    if (macType.isEmpty()) macType = root["addr_type"].as<String>();
    if (macType.isEmpty()) macType = root["type"].as<String>();
    macType.toLowerCase();
    macType.trim();

    outDevice.isRandomMac = (macType == "random" || macType == "rnd" || macType == "rand");

    JsonArray servicesArr = root["services"].as<JsonArray>();
    for (JsonObject svcObj : servicesArr) {
        HoneypotServiceDef sDef;
        sDef.uuid = normalizeHoneypotUuid(svcObj["uuid"].as<String>());
        if (sDef.uuid.isEmpty()) continue;
        if (!isValidHoneypotUuid(sDef.uuid)) {
            Serial.printf("[HONEYPOT-JSON] Invalid service UUID: %s\n", sDef.uuid.c_str());
            return false;
        }

        JsonArray charsArr = svcObj["characteristics"].as<JsonArray>();
        if (charsArr.isNull()) {
            charsArr = svcObj["attributes"].as<JsonArray>();
        }

        for (JsonObject charObj : charsArr) {
            HoneypotCharacteristicDef cDef;
            cDef.uuid = normalizeHoneypotUuid(charObj["uuid"].as<String>());
            if (cDef.uuid.isEmpty()) continue;
            if (!isValidHoneypotUuid(cDef.uuid)) {
                Serial.printf("[HONEYPOT-JSON] Invalid characteristic UUID: %s\n", cDef.uuid.c_str());
                return false;
            }

            cDef.properties = parseProperties(charObj["properties"]);
            if (cDef.properties == 0) {
                cDef.properties = parseProperties(charObj["props"]);
            }

            bool isHex = charObj["is_hex"].as<bool>() || charObj["hex"].as<bool>();
            String valStr = charObj["value"].as<String>();
            if (valStr.isEmpty()) {
                valStr = charObj["read_value"].as<String>();
            }
            if (valStr.isEmpty() && charObj["hex_value"].is<const char *>()) {
                valStr = charObj["hex_value"].as<String>();
                isHex = true;
            }

            cDef.rawValueStr = valStr;
            cDef.isHex = isHex;

            if (valStr.startsWith("0x") || valStr.startsWith("0X") || valStr.startsWith("hex:")) {
                isHex = true;
                cDef.isHex = true;
            }

            if (isHex && !valStr.isEmpty()) {
                cDef.readValue = parseHexStringToBytes(valStr);
            } else if (!valStr.isEmpty()) {
                cDef.readValue.assign(valStr.c_str(), valStr.c_str() + valStr.length());
            }

            if (cDef.properties == 0) {
                cDef.properties = NIMBLE_PROPERTY::READ;
            }

            sDef.characteristics.push_back(cDef);
        }

        outDevice.services.push_back(sDef);
    }

    return (!outDevice.services.empty());
}

static HoneypotDeviceDef getBuiltinAirohaRacePreset() {
    HoneypotDeviceDef d;
    d.name = "Airoha RACE Device";
    d.mac = "94:DB:56:AB:CD:EF";
    d.isRandomMac = false;

    // 1. Device Information Service (0x180A)
    HoneypotServiceDef disSvc;
    disSvc.uuid = "180A";

    auto addDisChar = [&disSvc](const char *uuid, const char *val) {
        HoneypotCharacteristicDef c;
        c.uuid = uuid;
        c.properties = NIMBLE_PROPERTY::READ;
        c.rawValueStr = val;
        c.isHex = false;
        c.readValue.assign(val, val + strlen(val));
        disSvc.characteristics.push_back(c);
    };

    addDisChar("2A29", "Airoha Technology"); // Manufacturer Name
    addDisChar("2A24", "AB1562A");           // Model Number
    addDisChar("2A25", "001B66814A2C");      // Serial Number
    addDisChar("2A26", "2.5.0");             // Firmware Revision
    addDisChar("2A27", "v1.0");              // Hardware Revision
    addDisChar("2A28", "SDK-2.5.1");         // Software Revision
    d.services.push_back(disSvc);

    // 2. Battery Service (0x180F)
    HoneypotServiceDef batSvc;
    batSvc.uuid = "180F";
    HoneypotCharacteristicDef batChar;
    batChar.uuid = "2A19";
    batChar.properties = NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY;
    batChar.rawValueStr = "5A";
    batChar.isHex = true;
    batChar.readValue.push_back(0x5A); // 90%
    batSvc.characteristics.push_back(batChar);
    d.services.push_back(batSvc);

    // 3. Airoha Standard RACE GATT Service (CVE-2025-20700 & CVE-2025-20701 target)
    HoneypotServiceDef raceSvc;
    raceSvc.uuid = "5052494D-2DAB-0341-6972-6F6861424C45";

    // TX Characteristic (Write / Write Without Response)
    HoneypotCharacteristicDef txChar;
    txChar.uuid = "43484152-2DAB-3241-6972-6F6861424C45";
    txChar.properties = NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR;
    txChar.rawValueStr = "";
    raceSvc.characteristics.push_back(txChar);

    // RX Characteristic (Read / Notify / Indicate)
    // Response payload simulating RACE return code 0x00 (SUCCESS)
    HoneypotCharacteristicDef rxChar;
    rxChar.uuid = "43484152-2DAB-3141-6972-6F6861424C45";
    rxChar.properties = NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE;
    rxChar.rawValueStr = "055B02000000";
    rxChar.isHex = true;
    rxChar.readValue = { 0x05, 0x5B, 0x02, 0x00, 0x00, 0x00 };
    raceSvc.characteristics.push_back(rxChar);

    // RX Alt Characteristic (Read / Notify)
    HoneypotCharacteristicDef rxAltChar;
    rxAltChar.uuid = "43484152-2DAB-3041-6972-6F6861424C45";
    rxAltChar.properties = NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY;
    rxAltChar.rawValueStr = "055B02000000";
    rxAltChar.isHex = true;
    rxAltChar.readValue = { 0x05, 0x5B, 0x02, 0x00, 0x00, 0x00 };
    raceSvc.characteristics.push_back(rxAltChar);

    d.services.push_back(raceSvc);
    return d;
}

static HoneypotDeviceDef getBuiltinSonyRacePreset() {
    HoneypotDeviceDef d;
    d.name = "WH-1000XM4";
    d.mac = "00:1B:66:81:4A:2C";
    d.isRandomMac = false;

    // Device Information
    HoneypotServiceDef disSvc;
    disSvc.uuid = "180A";

    auto addDisChar = [&disSvc](const char *uuid, const char *val) {
        HoneypotCharacteristicDef c;
        c.uuid = uuid;
        c.properties = NIMBLE_PROPERTY::READ;
        c.rawValueStr = val;
        c.isHex = false;
        c.readValue.assign(val, val + strlen(val));
        disSvc.characteristics.push_back(c);
    };

    addDisChar("2A29", "Sony Corporation");
    addDisChar("2A24", "WH-1000XM4");
    addDisChar("2A25", "001B66814A2C");
    addDisChar("2A26", "2.5.0");
    addDisChar("2A27", "v1.0");
    addDisChar("2A28", "SDK-2.5.1");
    d.services.push_back(disSvc);

    // Battery Service
    HoneypotServiceDef batSvc;
    batSvc.uuid = "180F";
    HoneypotCharacteristicDef batChar;
    batChar.uuid = "2A19";
    batChar.properties = NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY;
    batChar.rawValueStr = "5F";
    batChar.isHex = true;
    batChar.readValue.push_back(0x5F); // 95%
    batSvc.characteristics.push_back(batChar);
    d.services.push_back(batSvc);

    // Sony Vendor RACE Service
    HoneypotServiceDef sonySvc;
    sonySvc.uuid = "dc405470-a351-4a59-97d8-2e2e3b207fbb";

    HoneypotCharacteristicDef txChar;
    txChar.uuid = "bfd869fa-a3f2-4c2f-bcff-3eb1ec80cead";
    txChar.properties = NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR;
    txChar.rawValueStr = "";
    sonySvc.characteristics.push_back(txChar);

    HoneypotCharacteristicDef rxChar;
    rxChar.uuid = "2a6b6575-faf6-418c-923f-ccd63a56d955";
    rxChar.properties = NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE;
    rxChar.rawValueStr = "055B02000000";
    rxChar.isHex = true;
    rxChar.readValue = { 0x05, 0x5B, 0x02, 0x00, 0x00, 0x00 };
    sonySvc.characteristics.push_back(rxChar);

    d.services.push_back(sonySvc);
    return d;
}

static HoneypotDeviceDef getBuiltinAiroha16BitPreset() {
    HoneypotDeviceDef d;
    d.name = "Airoha Headset";
    d.mac = "4C:65:A8:12:34:56";
    d.isRandomMac = true;

    HoneypotServiceDef disSvc;
    disSvc.uuid = "180A";
    HoneypotCharacteristicDef mfgChar;
    mfgChar.uuid = "2A29";
    mfgChar.properties = NIMBLE_PROPERTY::READ;
    mfgChar.rawValueStr = "Airoha";
    mfgChar.readValue.assign("Airoha", "Airoha" + 6);
    disSvc.characteristics.push_back(mfgChar);
    d.services.push_back(disSvc);

    HoneypotServiceDef fef0Svc;
    fef0Svc.uuid = "0000fef0-0000-1000-8000-00805f9b34fb";

    HoneypotCharacteristicDef txChar;
    txChar.uuid = "0000fef1-0000-1000-8000-00805f9b34fb";
    txChar.properties = NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR;
    fef0Svc.characteristics.push_back(txChar);

    HoneypotCharacteristicDef rxChar;
    rxChar.uuid = "0000fef2-0000-1000-8000-00805f9b34fb";
    rxChar.properties = NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY;
    rxChar.rawValueStr = "055B02000000";
    rxChar.isHex = true;
    rxChar.readValue = { 0x05, 0x5B, 0x02, 0x00, 0x00, 0x00 };
    fef0Svc.characteristics.push_back(rxChar);

    d.services.push_back(fef0Svc);
    return d;
}

static HoneypotDeviceDef getBuiltinSmartLockPreset() {
    HoneypotDeviceDef d;
    d.name = "SmartLock-Pro-92";
    d.mac = "A4:C1:38:99:88:77";
    d.isRandomMac = false;

    HoneypotServiceDef disSvc;
    disSvc.uuid = "180A";
    HoneypotCharacteristicDef mfgChar;
    mfgChar.uuid = "2A29";
    mfgChar.properties = NIMBLE_PROPERTY::READ;
    mfgChar.rawValueStr = "SmartSecurity Inc";
    mfgChar.readValue.assign("SmartSecurity Inc", "SmartSecurity Inc" + 17);
    disSvc.characteristics.push_back(mfgChar);

    HoneypotCharacteristicDef modelChar;
    modelChar.uuid = "2A24";
    modelChar.properties = NIMBLE_PROPERTY::READ;
    modelChar.rawValueStr = "SL-9200";
    modelChar.readValue.assign("SL-9200", "SL-9200" + 7);
    disSvc.characteristics.push_back(modelChar);
    d.services.push_back(disSvc);

    HoneypotServiceDef lockSvc;
    lockSvc.uuid = "0000ffe0-0000-1000-8000-00805f9b34fb";

    HoneypotCharacteristicDef authChar;
    authChar.uuid = "0000ffe1-0000-1000-8000-00805f9b34fb";
    authChar.properties = NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE;
    authChar.rawValueStr = "41444D494E"; // "ADMIN"
    authChar.isHex = true;
    authChar.readValue = { 'A', 'D', 'M', 'I', 'N' };
    lockSvc.characteristics.push_back(authChar);

    HoneypotCharacteristicDef stateChar;
    stateChar.uuid = "0000ffe2-0000-1000-8000-00805f9b34fb";
    stateChar.properties = NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY;
    stateChar.rawValueStr = "01"; // Locked
    stateChar.isHex = true;
    stateChar.readValue = { 0x01 };
    lockSvc.characteristics.push_back(stateChar);

    d.services.push_back(lockSvc);
    return d;
}

static String serializeHoneypotDeviceToJson(const HoneypotDeviceDef &dev) {
    JsonDocument doc;
    doc["name"] = dev.name;
    doc["mac"] = dev.mac;
    doc["mac_type"] = dev.isRandomMac ? "random" : "public";

    JsonArray svcsArr = doc["services"].to<JsonArray>();
    for (const auto &svc : dev.services) {
        JsonObject sObj = svcsArr.add<JsonObject>();
        sObj["uuid"] = svc.uuid;

        JsonArray charsArr = sObj["characteristics"].to<JsonArray>();
        for (const auto &ch : svc.characteristics) {
            JsonObject cObj = charsArr.add<JsonObject>();
            cObj["uuid"] = ch.uuid;

            JsonArray propsArr = cObj["properties"].to<JsonArray>();
            if (ch.properties & NIMBLE_PROPERTY::READ) propsArr.add("read");
            if (ch.properties & NIMBLE_PROPERTY::WRITE) propsArr.add("write");
            if (ch.properties & NIMBLE_PROPERTY::WRITE_NR) propsArr.add("write_nr");
            if (ch.properties & NIMBLE_PROPERTY::NOTIFY) propsArr.add("notify");
            if (ch.properties & NIMBLE_PROPERTY::INDICATE) propsArr.add("indicate");

            if (ch.isHex) {
                cObj["value"] = ch.rawValueStr;
                cObj["is_hex"] = true;
            } else {
                cObj["value"] = ch.rawValueStr;
            }
        }
    }

    String outJson;
    serializeJsonPretty(doc, outJson);
    return outJson;
}

//=============================================================================
// Server & Characteristic Callbacks
//=============================================================================

class HoneypotServerCallbacks : public NimBLEServerCallbacks {
public:
    void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override {
        g_hpState.isConnected = true;
        g_hpState.connCount++;
        std::string pAddr = connInfo.getAddress().toString();
        uint16_t mtu = connInfo.getMTU();
        g_hpState.setPeer(pAddr.c_str(), mtu, connInfo.getConnHandle());
        g_hpState.addLog("[CONN] From " + String(pAddr.c_str()) + " (MTU:" + String(mtu) + ")");
        Serial.printf("[HONEYPOT] Incoming connection from %s (MTU: %d)\n", pAddr.c_str(), mtu);
    }

    void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override {
        g_hpState.isConnected = false;
        std::string peer = connInfo.getAddress().toString();
        g_hpState.setPeer("None", 23);
        g_hpState.addLog("[DISC] " + String(peer.c_str()) + " (0x" + String(reason, HEX) + ")");
        Serial.printf("[HONEYPOT] Disconnected by %s (Reason: 0x%02X)\n", peer.c_str(), reason);
    }

    void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo) override {
        char addr[20] = {0};
        g_hpState.getPeer(addr, sizeof(addr));
        g_hpState.setPeer(addr, MTU);
        g_hpState.addLog("[MTU] Updated: " + String(MTU) + " B");
        Serial.printf("[HONEYPOT] MTU updated to: %d\n", MTU);
    }
};

class HoneypotCharCallbacks : public NimBLECharacteristicCallbacks {
private:
    std::vector<uint8_t> m_readValue;

    void restorePresetValue(NimBLECharacteristic *pChar) {
        static const uint8_t emptyValue = 0;
        pChar->setValue(m_readValue.empty() ? &emptyValue : m_readValue.data(), m_readValue.size());
    }

public:
    explicit HoneypotCharCallbacks(const std::vector<uint8_t> &val = {}) : m_readValue(val) {}

    void onRead(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override {
        g_hpState.readCount++;
        String uuidStr = pChar->getUUID().toString().c_str();

        restorePresetValue(pChar);

        String shortU = shortUuidStr(uuidStr);
        size_t len = pChar->getValue().length();
        g_hpState.addLog("[READ] " + shortU + " (" + String((int)len) + "B)");
        Serial.printf("[HONEYPOT] Read on %s from %s (%d bytes returned)\n",
                      uuidStr.c_str(), connInfo.getAddress().toString().c_str(), (int)len);
    }

    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override {
        g_hpState.writeCount++;
        String uuidStr = pChar->getUUID().toString().c_str();
        std::string raw = pChar->getValue();
        const uint8_t *rawBytes = (const uint8_t *)raw.data();
        size_t rawLen = raw.length();

        String hexPayload = bytesToHexString(rawBytes, rawLen, 12);
        g_hpState.setLastWrite(hexPayload.c_str());

        String shortU = shortUuidStr(uuidStr);
        g_hpState.addLog("[WRITE] " + shortU + ": " + hexPayload);
        Serial.printf("[HONEYPOT] Write on %s from %s (%d bytes): %s\n",
                      uuidStr.c_str(), connInfo.getAddress().toString().c_str(), (int)rawLen, hexPayload.c_str());

        // In a honeypot, write operations have NO effect on internal state / system.
        // If the characteristic had a predefined read value, reset it so subsequent reads remain static.
        restorePresetValue(pChar);
    }

    void onSubscribe(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo, uint16_t subValue) override {
        g_hpState.subCount++;
        String uuidStr = pChar->getUUID().toString().c_str();
        String shortU = shortUuidStr(uuidStr);
        String action = (subValue == 0) ? "Unsub" : (subValue == 1) ? "NotifySub" : "IndicateSub";
        g_hpState.addLog("[" + action + "] " + shortU);
        Serial.printf("[HONEYPOT] Subscription on %s: %s\n", uuidStr.c_str(), action.c_str());
    }

};

static HoneypotServerCallbacks g_hpServerCallbacks;
static std::vector<std::unique_ptr<HoneypotCharCallbacks>> g_hpCharCallbacksPool;

//=============================================================================
// Service Lifecycle
//=============================================================================

bool startGattHoneypotService(const String &jsonConfigOrPath) {
    if (g_hpState.isRunning) {
        return true;
    }
    g_hpState.reset();

    HoneypotDeviceDef dev;
    bool loaded = false;

    if (!jsonConfigOrPath.isEmpty()) {
        // Check if string is a JSON file path
        if (jsonConfigOrPath.startsWith("/") || jsonConfigOrPath.endsWith(".json")) {
            bool useSd = sdcardMounted;
            if (!useSd) useSd = setupSdCard(2);

            FS *fs = useSd ? (FS *)&SD : (FS *)&LittleFS;
            String path = jsonConfigOrPath;
            if (useSd && path.startsWith("/sd")) {
                path = path.substring(3);
            }

            if (fs->exists(path)) {
                File f = fs->open(path, FILE_READ);
                if (f) {
                    String content = f.readString();
                    f.close();
                    loaded = parseHoneypotJson(content, dev);
                }
            }
        }

        // If not loaded yet, try parsing as raw JSON string
        if (!loaded) {
            loaded = parseHoneypotJson(jsonConfigOrPath, dev);
        }
    }

    if (!loaded) return false;

    g_hpState.activeDevice = dev;

    // Preserve the interface address so a custom profile does not leak into
    // other BLE features after the honeypot stops.
    uint8_t mac[6];
    bool hasCustomMac = parseMacBytes(dev.mac, mac);
    if (!hasCustomMac) return false;
    if (esp_read_mac(g_originalBluetoothMac, ESP_MAC_BT) != ESP_OK) return false;
    g_originalBluetoothMacSaved = true;

    // NimBLE reads the public identity address during initialization, so set
    // the profile address before starting the stack.
    if (!dev.isRandomMac && esp_iface_mac_addr_set(mac, ESP_MAC_BT) != ESP_OK) {
        g_originalBluetoothMacSaved = false;
        return false;
    }

    // Initialize BLE only after configuring the public identity address.
    if (!BLEStateManager::initBLE(dev.name, ESP_PWR_LVL_P9)) {
        esp_iface_mac_addr_set(g_originalBluetoothMac, ESP_MAC_BT);
        g_originalBluetoothMacSaved = false;
        return false;
    }

    if (dev.isRandomMac) {
        uint8_t addr_le[6];
        addr_le[0] = mac[5];
        addr_le[1] = mac[4];
        addr_le[2] = mac[3];
        addr_le[3] = mac[2];
        addr_le[4] = mac[1];
        addr_le[5] = mac[0] | 0xC0; // MSB with static random bits set
        if (ble_hs_id_set_rnd(addr_le) != 0 || !NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM)) {
            esp_iface_mac_addr_set(g_originalBluetoothMac, ESP_MAC_BT);
            NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
            BLEStateManager::deinitBLE(true);
            g_originalBluetoothMacSaved = false;
            return false;
        }
    } else if (!NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC)) {
        esp_iface_mac_addr_set(g_originalBluetoothMac, ESP_MAC_BT);
        BLEStateManager::deinitBLE(true);
        g_originalBluetoothMacSaved = false;
        return false;
    }

    NimBLEDevice::setSecurityAuth(false, false, false);

    // 3. Create Server
    NimBLEServer *pServer = NimBLEDevice::createServer();
    if (!pServer) {
        esp_iface_mac_addr_set(g_originalBluetoothMac, ESP_MAC_BT);
        NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
        BLEStateManager::deinitBLE(true);
        g_originalBluetoothMacSaved = false;
        return false;
    }

    pServer->setCallbacks(&g_hpServerCallbacks, false);
    pServer->advertiseOnDisconnect(true);

    // 4. Build GATT database from device definition
    size_t totalChars = 0;
    for (const auto &svc : dev.services) {
        totalChars += svc.characteristics.size();
    }
    g_hpCharCallbacksPool.clear();
    g_hpCharCallbacksPool.reserve(totalChars + 8);

    NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
    pAdv->reset();
    if (!pAdv->setConnectableMode(BLE_GAP_CONN_MODE_UND) ||
        !pAdv->setDiscoverableMode(BLE_GAP_DISC_MODE_GEN)) {
        if (g_originalBluetoothMacSaved) {
            esp_iface_mac_addr_set(g_originalBluetoothMac, ESP_MAC_BT);
            g_originalBluetoothMacSaved = false;
        }
        NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
        BLEStateManager::deinitBLE(true);
        return false;
    }
    // Put the device name in the primary packet so passive scanners can
    // identify the honeypot; service UUIDs can spill into the scan response.
    String advertisedName = dev.name;
    if (advertisedName.length() > 26) advertisedName = advertisedName.substring(0, 26);
    if (!pAdv->setName(advertisedName.c_str())) {
        Serial.println(F("[HONEYPOT] Failed to add device name to advertising data"));
        if (g_originalBluetoothMacSaved) {
            esp_iface_mac_addr_set(g_originalBluetoothMac, ESP_MAC_BT);
            g_originalBluetoothMacSaved = false;
        }
        NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
        BLEStateManager::deinitBLE(true);
        g_hpCharCallbacksPool.clear();
        return false;
    }
    pAdv->enableScanResponse(true);

    for (size_t sIdx = 0; sIdx < dev.services.size(); sIdx++) {
        const auto &sDef = dev.services[sIdx];
        if (isNimbleManagedServiceUuid(sDef.uuid)) {
            g_hpState.addLog("[GATT] Using stack-managed service " + sDef.uuid);
            continue;
        }

        NimBLEService *pSvc = pServer->createService(sDef.uuid.c_str());
        if (!pSvc) continue;

        // NimBLE fills the advertising packet first, then the scan response.
        // A UUID that does not fit either packet remains available over GATT.
        if (!pAdv->addServiceUUID(sDef.uuid.c_str())) {
            g_hpState.addLog("[ADV] Service UUID list full");
        }

        for (size_t cIdx = 0; cIdx < sDef.characteristics.size(); cIdx++) {
            const auto &cDef = sDef.characteristics[cIdx];
            NimBLECharacteristic *pChar = pSvc->createCharacteristic(
                cDef.uuid.c_str(),
                cDef.properties
            );
            if (pChar) {
                if (!cDef.readValue.empty()) {
                    pChar->setValue(cDef.readValue.data(), cDef.readValue.size());
                }

                g_hpCharCallbacksPool.emplace_back(std::make_unique<HoneypotCharCallbacks>(cDef.readValue));
                pChar->setCallbacks(g_hpCharCallbacksPool.back().get());
            }
        }
    }

    if (!pServer->start()) {
        if (g_originalBluetoothMacSaved) {
            esp_iface_mac_addr_set(g_originalBluetoothMac, ESP_MAC_BT);
            g_originalBluetoothMacSaved = false;
        }
        NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
        BLEStateManager::deinitBLE(true);
        g_hpCharCallbacksPool.clear();
        return false;
    }

    // 5. Start connectable advertising indefinitely.
    if (!pAdv->start(0)) {
        Serial.println(F("[HONEYPOT] Failed to start advertising"));
        pAdv->stop();
        if (g_originalBluetoothMacSaved) {
            esp_iface_mac_addr_set(g_originalBluetoothMac, ESP_MAC_BT);
            g_originalBluetoothMacSaved = false;
        }
        NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
        BLEStateManager::deinitBLE(true);
        g_hpCharCallbacksPool.clear();
        return false;
    }
    g_hpState.isRunning = true;

    g_hpState.addLog("[INIT] Honeypot: " + dev.name);
    g_hpState.addLog("[MAC] " + dev.mac + (dev.isRandomMac ? " (RND)" : " (PUB)"));
    g_hpState.addLog("[ADV] Advertising active");
    Serial.printf("[HONEYPOT] Started '%s' (%s, %s)\n",
                  dev.name.c_str(), dev.mac.c_str(), dev.isRandomMac ? "RANDOM" : "PUBLIC");
    return true;
}

void stopGattHoneypotService() {
    if (!g_hpState.isRunning) return;
    g_hpState.isRunning = false;
    g_hpState.addLog("[STOP] Honeypot shutting down...");

    NimBLEServer *pServer = NimBLEDevice::getServer();
    if (pServer) {
        if (pServer->getAdvertising()) {
            pServer->getAdvertising()->stop();
        }
        std::vector<uint16_t> peers = pServer->getPeerDevices();
        for (uint16_t handle : peers) {
            pServer->disconnect(handle);
        }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
    // NimBLE address APIs require a live host; restore identity before
    // deinitializing the stack. Keep callbacks alive until that teardown ends.
    if (g_originalBluetoothMacSaved) {
        esp_iface_mac_addr_set(g_originalBluetoothMac, ESP_MAC_BT);
        g_originalBluetoothMacSaved = false;
    }
    NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
    BLEStateManager::deinitBLE(true);
    g_hpCharCallbacksPool.clear();
    Serial.println(F("[HONEYPOT] Stopped and BLE stack cleaned."));
}

bool isGattHoneypotActive() {
    return g_hpState.isRunning;
}

String getGattHoneypotStatus() {
    String status = "=== GATT Honeypot Status ===\nState: ";
    status += g_hpState.isRunning ? "RUNNING" : "STOPPED";
    status += "\nProfile: ";
    status += g_hpState.activeDevice.name.isEmpty() ? "(none)" : g_hpState.activeDevice.name;
    if (!g_hpState.activeDevice.mac.isEmpty()) {
        status += "\nIdentity: ";
        status += g_hpState.activeDevice.mac;
        status += g_hpState.activeDevice.isRandomMac ? " (random)" : " (public)";
    }

    bool advertising = false;
    if (g_hpState.isRunning) {
        NimBLEServer *pServer = NimBLEDevice::getServer();
        NimBLEAdvertising *pAdv = pServer ? pServer->getAdvertising() : nullptr;
        advertising = pAdv && pAdv->isAdvertising();
    }
    status += "\nAdvertising: ";
    status += advertising ? "ACTIVE" : "INACTIVE";
    status += "\nConnected: ";
    status += g_hpState.isConnected ? "YES" : "NO";

    char peer[20] = "None";
    uint16_t mtu = 23;
    g_hpState.getPeer(peer, sizeof(peer), &mtu);
    status += "\nPeer: ";
    status += peer;
    status += " (MTU ";
    status += String(mtu);
    status += ")\nConnections: ";
    status += String((unsigned int)g_hpState.connCount);
    status += "\nReads: ";
    status += String((unsigned int)g_hpState.readCount);
    status += "  Writes: ";
    status += String((unsigned int)g_hpState.writeCount);
    status += "  Subscriptions: ";
    status += String((unsigned int)g_hpState.subCount);
    return status;
}

String getGattHoneypotLogs() {
    std::vector<String> snapshot;
    g_hpState.initMutex();
    if (g_hpState.logMutex && xSemaphoreTake(g_hpState.logMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        snapshot.assign(g_hpState.logLines.begin(), g_hpState.logLines.end());
        xSemaphoreGive(g_hpState.logMutex);
    }

    String logs = "=== Recent GATT Honeypot Events ===\n";
    if (snapshot.empty()) {
        logs += "(no events)";
        return logs;
    }
    for (const String &line : snapshot) {
        logs += line;
        logs += '\n';
    }
    return logs;
}

bool setGattHoneypotAdvertising(bool enabled) {
    if (!g_hpState.isRunning) return false;

    NimBLEServer *pServer = NimBLEDevice::getServer();
    NimBLEAdvertising *pAdv = pServer ? pServer->getAdvertising() : nullptr;
    if (!pAdv) return false;

    if (enabled) {
        if (g_hpState.isConnected) return false;
        if (pAdv->isAdvertising()) return true;
        if (!pAdv->start(0)) return false;
        g_hpState.addLog("[ADV] Advertising started by serial command");
        return true;
    }

    pAdv->stop();
    g_hpState.addLog("[ADV] Advertising stopped by serial command");
    return !pAdv->isAdvertising();
}

//=============================================================================
// Interactive Live Monitor UI
//=============================================================================

void runGattHoneypot(const String &jsonFilePath) {
    if (!startGattHoneypotService(jsonFilePath)) {
        displayError("Failed to start Honeypot", true);
        return;
    }

    tft.fillScreen(bruceConfig.bgColor);
    drawMainBorderWithTitle("GATT HONEYPOT");

    int lineH = 8 * FP + 3;
    int headerY = BORDER_PAD_Y + 14;
    int statsY = headerY + lineH + 2;
    int logBoxTop = statsY + lineH + 4;
    int footY = tftHeight - BORDER_PAD_Y - 9;
    int logBoxH = footY - logBoxTop - 2;
    int logVisibleRows = logBoxH / lineH;
    if (logVisibleRows < 1) logVisibleRows = 1;

    uint32_t lastRefresh = 0;
    bool lastConnected = false;
    uint32_t lastReadCount = 0xFFFFFFFF;
    uint32_t lastWriteCount = 0xFFFFFFFF;
    uint32_t lastLogGeneration = UINT32_MAX;
    uint32_t lastAdvertisingCheck = 0;
    while (g_hpState.isRunning) {
        if (check(EscPress) || check(PrevPress)) {
            break;
        }

        if (check(SelPress)) {
            std::vector<Option> actions = {
                {"Stop Honeypot", []() {}},
                {"Clear Text", []() {}}
            };
            bool connected = g_hpState.isConnected;
            if (connected) actions.push_back({"Track Connected Device", []() {}});
            actions.push_back({"< Back", []() {}});

            int selected = loopOptions(actions, MENU_TYPE_SUBMENU, "HONEYPOT ACTIONS");
            if (selected == 0) {
                break;
            } else if (selected == 1) {
                g_hpState.initMutex();
                if (g_hpState.logMutex && xSemaphoreTake(g_hpState.logMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    g_hpState.logLines.clear();
                    g_hpState.logGeneration++;
                    xSemaphoreGive(g_hpState.logMutex);
                }
                lastRefresh = 0;
            } else if (connected && selected == 2 && g_hpState.isConnected) {
                char peerAddress[20] = "None";
                uint16_t peerConnHandle = 0xFFFF;
                g_hpState.getPeer(peerAddress, sizeof(peerAddress), nullptr, &peerConnHandle);
                if (peerConnHandle != 0xFFFF) {
                    bleTrackerRun(peerAddress, "Connected: " + String(peerAddress), nullptr, 0xFF, peerConnHandle);
                } else {
                    displayWarning("Connected peer unavailable", true);
                }
            }
            tft.fillScreen(bruceConfig.bgColor);
            drawMainBorderWithTitle("GATT HONEYPOT");
            lastRefresh = 0;
            lastLogGeneration = UINT32_MAX;
        }

        keyStroke k = _getKeyPress();
        if (k.pressed || !k.word.empty()) {
            if (k.del || k.exit_key) {
                break;
            }
            for (char ch : k.word) {
                char lower = tolower(ch);
                if (lower == 'c') {
                    // Clear log
                    g_hpState.initMutex();
                    if (g_hpState.logMutex && xSemaphoreTake(g_hpState.logMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                        g_hpState.logLines.clear();
                        xSemaphoreGive(g_hpState.logMutex);
                    }
                    g_hpState.logGeneration++;
                }
            }
        }

        uint32_t now = millis();
        if (!g_hpState.isConnected && now - lastAdvertisingCheck >= 1000) {
            lastAdvertisingCheck = now;
            NimBLEServer *pServer = NimBLEDevice::getServer();
            NimBLEAdvertising *pAdv = pServer ? pServer->getAdvertising() : nullptr;
            if (pAdv && !pAdv->isAdvertising()) {
                if (pAdv->start(0)) {
                    g_hpState.addLog("[ADV] Advertising resumed");
                    Serial.println(F("[HONEYPOT] Advertising resumed"));
                } else {
                    g_hpState.addLog("[ERR] Advertising restart failed");
                    Serial.println(F("[HONEYPOT] Advertising restart failed"));
                }
            }
        }
        if (now - lastRefresh >= 200) {
            lastRefresh = now;

            // 1. Device Info Header
            tft.setTextSize(FP);
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);

            String titleLine = "Trap: " + g_hpState.activeDevice.name;
            if (titleLine.length() > 22) titleLine = titleLine.substring(0, 20) + "..";
            titleLine += " [" + String(g_hpState.activeDevice.isRandomMac ? "RND" : "PUB") + "]";

            tft.fillRect(BORDER_PAD_X, headerY, tftWidth - 2 * BORDER_PAD_X, lineH, bruceConfig.bgColor);
            tft.drawString(titleLine, BORDER_PAD_X, headerY);

            // 2. Connection & Activity Stats
            char peerBuf[20] = {0};
            uint16_t peerMtu = 23;
            g_hpState.getPeer(peerBuf, sizeof(peerBuf), &peerMtu);

            String statusStr;
            if (g_hpState.isConnected) {
                statusStr = "CONN:" + String(peerBuf).substring(9) + " R:" + String(g_hpState.readCount) + " W:" + String(g_hpState.writeCount);
            } else {
                statusStr = "WAITING.. Conns:" + String(g_hpState.connCount) + " R:" + String(g_hpState.readCount) + " W:" + String(g_hpState.writeCount);
            }

            tft.fillRect(BORDER_PAD_X, statsY, tftWidth - 2 * BORDER_PAD_X, lineH, bruceConfig.bgColor);
            tft.setTextColor(g_hpState.isConnected ? TFT_GREEN : bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString(statusStr, BORDER_PAD_X, statsY);

            // 3. Log View Area
            std::vector<String> snapshotLogs;
            uint32_t snapshotLogGeneration = 0;
            g_hpState.initMutex();
            if (g_hpState.logMutex && xSemaphoreTake(g_hpState.logMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
                snapshotLogs.assign(g_hpState.logLines.begin(), g_hpState.logLines.end());
                snapshotLogGeneration = g_hpState.logGeneration;
                xSemaphoreGive(g_hpState.logMutex);
            }

            if (snapshotLogGeneration != lastLogGeneration || g_hpState.isConnected != lastConnected ||
                g_hpState.readCount != lastReadCount || g_hpState.writeCount != lastWriteCount) {
                lastLogGeneration = snapshotLogGeneration;
                lastConnected = g_hpState.isConnected;
                lastReadCount = g_hpState.readCount;
                lastWriteCount = g_hpState.writeCount;

                tft.fillRect(BORDER_PAD_X, logBoxTop, tftWidth - 2 * BORDER_PAD_X, logBoxH, bruceConfig.bgColor);
                tft.drawFastHLine(BORDER_PAD_X, logBoxTop - 2, tftWidth - 2 * BORDER_PAD_X, bruceConfig.priColor);

                int logStartIdx = (int)snapshotLogs.size() - logVisibleRows;
                if (logStartIdx < 0) logStartIdx = 0;

                for (int r = 0; r < logVisibleRows && (logStartIdx + r) < (int)snapshotLogs.size(); r++) {
                    String line = snapshotLogs[logStartIdx + r];
                    uint16_t col = bruceConfig.priColor;
                    if (line.startsWith("[WRITE]")) col = TFT_RED;
                    else if (line.startsWith("[READ]")) col = TFT_YELLOW;
                    else if (line.startsWith("[CONN]")) col = TFT_GREEN;
                    else if (line.startsWith("[DISC]")) col = TFT_DARKGREY;

                    tft.setTextColor(col, bruceConfig.bgColor);
                    tft.drawString(line.substring(0, 32), BORDER_PAD_X, logBoxTop + r * lineH);
                }
            }

            // Footer
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString("SEL:Menu ESC:Stop C:Clear S:Save", BORDER_PAD_X, footY);
        }

        vTaskDelay(30 / portTICK_PERIOD_MS);
    }

    stopGattHoneypotService();
}

//=============================================================================
// Preset Selection & Menu System
//=============================================================================

static void showHoneypotDeviceDetailsUi(const HoneypotDeviceDef &dev) {
    ScrollableTextArea area(dev.name);
    area.addLine("=== Honeypot Profile ===");
    area.addLine("Name: " + dev.name);
    area.addLine("MAC:  " + dev.mac + (dev.isRandomMac ? " (Random)" : " (Public)"));
    area.addLine("Services: " + String((int)dev.services.size()));
    area.addLine("");

    for (size_t s = 0; s < dev.services.size(); s++) {
        const auto &svc = dev.services[s];
        area.addLine("Svc [" + String((int)s + 1) + "]: " + svc.uuid);
        for (size_t c = 0; c < svc.characteristics.size(); c++) {
            const auto &ch = svc.characteristics[c];
            String pStr = "";
            if (ch.properties & NIMBLE_PROPERTY::READ) pStr += "R ";
            if (ch.properties & NIMBLE_PROPERTY::WRITE) pStr += "W ";
            if (ch.properties & NIMBLE_PROPERTY::WRITE_NR) pStr += "WNR ";
            if (ch.properties & NIMBLE_PROPERTY::NOTIFY) pStr += "N ";
            if (ch.properties & NIMBLE_PROPERTY::INDICATE) pStr += "I ";
            pStr.trim();

            area.addLine("  Char: " + shortUuidStr(ch.uuid) + " [" + pStr + "]");
            if (!ch.rawValueStr.isEmpty()) {
                area.addLine("    Val: " + ch.rawValueStr);
            }
        }
        area.addLine("");
    }

    area.addLine("Press ESC to exit");
    area.show();
}

static void selectBuiltinPresetMenu() {
    struct PresetItem {
        String title;
        std::function<HoneypotDeviceDef()> getter;
    };

    std::vector<PresetItem> presets = {
        { "1. Airoha RACE (AB1562A)", getBuiltinAirohaRacePreset },
        { "2. Sony WH-1000XM4",      getBuiltinSonyRacePreset },
        { "3. Airoha 16-bit FEF0",   getBuiltinAiroha16BitPreset },
        { "4. Smart Lock IoT (FFE0)",getBuiltinSmartLockPreset }
    };

    int cursor = 0;
    while (true) {
        std::vector<Option> optList;
        for (const auto &p : presets) {
            optList.push_back({p.title.c_str(), []() {}});
        }
        optList.push_back({"< Back", []() {}});

        int sel = loopOptions(optList, MENU_TYPE_SUBMENU, "HONEYPOT PRESETS", cursor, false);
        if (sel < 0 || sel >= (int)presets.size()) {
            break;
        }

        HoneypotDeviceDef picked = presets[sel].getter();
        showHoneypotDeviceDetailsUi(picked);

        String jsonStr = serializeHoneypotDeviceToJson(picked);
        runGattHoneypot(jsonStr);
        break;
    }
}

static void loadJsonFileMenu() {
    bool useSd = sdcardMounted;
    if (!useSd) useSd = setupSdCard(2);

    FS *fs = useSd ? (FS *)&SD : (FS *)&LittleFS;
    String pickedFile = loopSD(*fs, true, "json", "/bruce/honeypot");
    if (pickedFile.isEmpty() || pickedFile == "/") {
        pickedFile = loopSD(*fs, true, "json", "/");
    }

    if (pickedFile.isEmpty() || pickedFile == "/") {
        displayWarning("No file selected", true);
        return;
    }

    File f = fs->open(pickedFile, FILE_READ);
    if (!f) {
        displayError("Cannot open file", true);
        return;
    }

    String content = f.readString();
    f.close();

    HoneypotDeviceDef dev;
    if (!parseHoneypotJson(content, dev)) {
        displayError("Invalid Honeypot JSON", true);
        return;
    }

    runGattHoneypot(pickedFile);
}

void gattHoneypotMenu() {
    int cursor = 0;
    while (true) {
        std::vector<Option> menuOpts = {
            {"Load JSON Device Profile (SD/LittleFS)", []() { loadJsonFileMenu(); }},
            {"< Back to Bluetooth Menu", []() {}}
        };

        int chosen = loopOptions(menuOpts, MENU_TYPE_SUBMENU, "GATT HONEYPOT", cursor, false);
        if (chosen < 0 || chosen == (int)menuOpts.size() - 1) {
            break;
        }
    }
}

#endif // !LITE_VERSION
