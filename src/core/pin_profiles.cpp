#include "pin_profiles.h"
#include "configPins.h"
#include "sd_functions.h"
#include <globals.h>
#include <algorithm>
#include <cstring>

static const char *const SPI_PIN_KEYS[] = {"sck", "miso", "mosi", "cs", "io0", "io1", "io2"};
static const char *const I2C_PIN_KEYS[] = {"sda", "scl"};
static const char *const UART_PIN_KEYS[] = {"rx", "tx"};
static const char *const IR_PIN_KEYS[] = {"tx", "rx"};
static const char *const SCALAR_PIN_KEYS[] = {"rfTx", "rfRx", "iButton"};
static const char *const BUS_PROFILE_KEYS[] = {
    "CC1101_Pins", "NRF24_Pins", "PN532_Pins", "SDCard_Pins", "IR_Pins", "uart_bus", "GPS_bus",
#if !(defined(SOC_HP_I2C_NUM) && SOC_HP_I2C_NUM < 2 && SYS_I2C_SDA >= 0 && SYS_I2C_SCL >= 0 && \
      !defined(BRUCE_BOARD_HAS_SOFTWARE_I2C))
    "i2c_bus",
#endif
#if !defined(LITE_VERSION)
    "W5500_Pins", "LoRa_Pins", "ST25R_Pins",
#endif
};

static bool isPinValue(JsonVariantConst value) {
    if (!value.is<int>()) return false;
    int pin = value.as<int>();
    return pin >= (int)GPIO_NUM_NC && pin < (int)GPIO_NUM_MAX;
}

static bool isPinObject(JsonVariantConst value, const char *const *keys, size_t keyCount) {
    if (!value.is<JsonObjectConst>()) return false;

    JsonObjectConst object = value.as<JsonObjectConst>();
    bool hasPin = false;
    for (size_t i = 0; i < keyCount; i++) {
        JsonVariantConst pin = object[keys[i]];
        if (pin.isNull()) continue;
        if (!isPinValue(pin)) return false;
        hasPin = true;
    }
    return hasPin;
}

static void scanRoot(FS &fs, const String &storageName, std::vector<PinProfile> &profiles) {
    File root = fs.open("/");
    if (!root || !root.isDirectory()) return;

    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            String path = file.name();
            if (path.endsWith(".pins")) {
                if (!path.startsWith("/")) path = "/" + path;
                String filename = path.substring(path.lastIndexOf('/') + 1);
                String name = filename.substring(0, filename.length() - 5);
                profiles.push_back({&fs, path, storageName + ": " + name});
            }
        }
        file.close();
        file = root.openNextFile();
    }
    root.close();
}

std::vector<PinProfile> scanPinProfiles() {
    std::vector<PinProfile> profiles;
    scanRoot(LittleFS, "LittleFS", profiles);
    if (sdcardMounted) scanRoot(SD, "SD", profiles);
    std::sort(profiles.begin(), profiles.end(), [](const PinProfile &left, const PinProfile &right) {
        return left.displayName.compareTo(right.displayName) < 0;
    });
    return profiles;
}

static bool readJson(FS &fs, const String &path, JsonDocument &doc, String &error) {
    File file = fs.open(path, FILE_READ);
    if (!file) {
        error = "Unable to open " + path;
        return false;
    }
    if (file.size() > 8192) {
        file.close();
        error = "File too large: " + path;
        return false;
    }

    DeserializationError result = deserializeJson(doc, file);
    file.close();
    if (result || !doc.is<JsonObject>()) {
        error = "Invalid JSON in " + path;
        return false;
    }
    return true;
}

static bool applyHardwarePins(JsonObject config, JsonObjectConst profilePins, String &error) {
    bool hasPins = false;
    JsonVariantConst legacyIrTx = profilePins["irTx"];
    JsonVariantConst legacyIrRx = profilePins["irRx"];
    bool hasIrProfile = profilePins["IR_Pins"].is<JsonObject>();
    if (!legacyIrTx.isNull() || !legacyIrRx.isNull()) {
        if ((!legacyIrTx.isNull() && !isPinValue(legacyIrTx)) || (!legacyIrRx.isNull() && !isPinValue(legacyIrRx))) {
            error = "Invalid legacy IR pin value";
            return false;
        }
        JsonObject irPins;
        if (config["IR_Pins"].is<JsonObject>()) irPins = config["IR_Pins"].as<JsonObject>();
        else irPins = config["IR_Pins"].to<JsonObject>();
        if (!legacyIrTx.isNull()) irPins["tx"].set(legacyIrTx);
        if (!legacyIrRx.isNull()) irPins["rx"].set(legacyIrRx);
        hasPins = true;
    }

    for (const char *key : SCALAR_PIN_KEYS) {
        JsonVariantConst value = profilePins[key];
        if (value.isNull()) continue;
        if (!isPinValue(value)) {
            error = String("Invalid pin value: ") + key;
            return false;
        }
        config[key].set(value);
        hasPins = true;
    }

    for (const char *key : BUS_PROFILE_KEYS) {
        JsonVariantConst value = profilePins[key];
        if (value.isNull()) continue;

        bool valid = false;
        if (strcmp(key, "i2c_bus") == 0)
            valid = isPinObject(value, I2C_PIN_KEYS, sizeof(I2C_PIN_KEYS) / sizeof(I2C_PIN_KEYS[0]));
        else if (strcmp(key, "IR_Pins") == 0)
            valid = isPinObject(value, IR_PIN_KEYS, sizeof(IR_PIN_KEYS) / sizeof(IR_PIN_KEYS[0]));
        else if (strcmp(key, "uart_bus") == 0 || strcmp(key, "GPS_bus") == 0)
            valid = isPinObject(value, UART_PIN_KEYS, sizeof(UART_PIN_KEYS) / sizeof(UART_PIN_KEYS[0]));
        else valid = isPinObject(value, SPI_PIN_KEYS, sizeof(SPI_PIN_KEYS) / sizeof(SPI_PIN_KEYS[0]));

        if (!valid) {
            error = String("Invalid pin map: ") + key;
            return false;
        }
        config[key].set(value);
        hasPins = true;
    }

    if (hasIrProfile || !legacyIrTx.isNull() || !legacyIrRx.isNull()) {
        config.remove("irTx");
        config.remove("irRx");
    }

    if (!hasPins) error = "No hardware pin fields found";
    return hasPins;
}

static bool writeJson(FS &fs, const char *path, JsonDocument &doc, String &error) {
    String temporaryPath = String(path) + ".tmp";
    String backupPath = String(path) + ".bak";
    if (fs.exists(temporaryPath)) fs.remove(temporaryPath);

    File file = fs.open(temporaryPath, FILE_WRITE);
    if (!file) {
        error = String("Unable to write ") + temporaryPath;
        return false;
    }

    size_t expected = measureJsonPretty(doc);
    size_t written = serializeJsonPretty(doc, file);
    file.flush();
    file.close();
    if (written != expected || written < 5) {
        fs.remove(temporaryPath);
        error = String("Failed to save ") + path;
        return false;
    }

    if (fs.exists(backupPath)) fs.remove(backupPath);
    bool hadOriginal = fs.exists(path);
    if (hadOriginal && !fs.rename(path, backupPath)) {
        fs.remove(temporaryPath);
        error = String("Unable to replace ") + path;
        return false;
    }
    if (!fs.rename(temporaryPath, path)) {
        if (hadOriginal) fs.rename(backupPath, path);
        fs.remove(temporaryPath);
        error = String("Unable to replace ") + path;
        return false;
    }
    if (hadOriginal) fs.remove(backupPath);
    return true;
}

bool applyPinProfile(const PinProfile &profile, String &error) {
    JsonDocument profileDoc;
    if (!readJson(*profile.fs, profile.path, profileDoc, error)) return false;

    JsonObjectConst profileRoot = profileDoc.as<JsonObjectConst>();
    String mac = getMacAddress();
    JsonObjectConst profilePins = profileRoot;
    if (profileRoot[mac].is<JsonObjectConst>()) profilePins = profileRoot[mac].as<JsonObjectConst>();

    FS *configSource;
    if (sdcardMounted && SD.exists(bruceConfigPins.filepath)) configSource = &SD;
    else configSource = &LittleFS;
    JsonDocument configDoc;
    if (!readJson(*configSource, bruceConfigPins.filepath, configDoc, error)) return false;

    JsonObject configRoot = configDoc[mac].as<JsonObject>();
    if (configRoot.isNull()) {
        error = "Current device config not found";
        return false;
    }
    if (!applyHardwarePins(configRoot, profilePins, error)) return false;

    if (!writeJson(LittleFS, bruceConfigPins.filepath, configDoc, error)) return false;
    if (sdcardMounted && !writeJson(SD, bruceConfigPins.filepath, configDoc, error)) return false;
    return true;
}