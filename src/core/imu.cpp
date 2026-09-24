#include "imu.h"
#include "core/bus_HAL.h"
#include "core/configPins.h"
#include "globals.h"
#include "imu_bmi270_config.h"
#include <Wire.h>
#include <math.h>

namespace {

// BMI270 I2C address depends on how the board wires SDO/AD0: 0x68 when pulled low (the more
// common default), 0x69 when pulled high. imu_detect() tries both and remembers whichever one
// actually answers, so this driver doesn't need a compile-time assumption per board.
constexpr uint8_t BMI270_I2C_ADDR_LOW = 0x68;
constexpr uint8_t BMI270_I2C_ADDR_HIGH = 0x69;
uint8_t bmi270Addr = BMI270_I2C_ADDR_LOW;

// Registers actually needed for detection, the mandatory config-file upload, and a coarse
// heading estimate - see imu.h's header comment for why the upload step exists at all.
constexpr uint8_t REG_CHIP_ID = 0x00;
constexpr uint8_t REG_ACC_DATA = 0x0C; // 6 bytes: acc_x, acc_y, acc_z (little-endian, signed)
constexpr uint8_t REG_GYR_DATA = 0x12; // 6 bytes: gyr_x, gyr_y, gyr_z (little-endian, signed)
constexpr uint8_t REG_GYR_CONF = 0x42;
constexpr uint8_t REG_GYR_RANGE = 0x43;
constexpr uint8_t REG_INTERNAL_STATUS = 0x21;
constexpr uint8_t REG_INIT_CTRL = 0x59;
constexpr uint8_t REG_INIT_ADDR_0 = 0x5B;
constexpr uint8_t REG_INIT_ADDR_1 = 0x5C;
constexpr uint8_t REG_INIT_DATA = 0x5E;
constexpr uint8_t REG_PWR_CONF = 0x7C;
constexpr uint8_t REG_PWR_CTRL = 0x7D;
constexpr uint8_t REG_CMD = 0x7E;

constexpr uint8_t BMI270_CHIP_ID = 0x24;
constexpr uint8_t CMD_SOFTRESET = 0xB6;

// GYR_RANGE = 0x00 -> +/-2000 dps (BMI270 reset default), giving this sensitivity in
// degrees-per-second per raw LSB (16-bit signed reading over the +/-2000 dps range).
constexpr float GYRO_DPS_PER_LSB = 2000.0f / 32768.0f;

bool detected = false;
bool initialized = false;
float headingDeg = 0.0f;
float gyroZOffset = 0.0f;
uint32_t lastSampleMs = 0;

// Both wrapped with lockSysI2CBus()/unlockSysI2CBus(): on boards where sys_i2c is a plain
// TwoWire (e.g. Cardputer ADV's Wire1, shared with the TCA8418 keyboard poll running on a
// different task) this is the only thing serializing the two - see lockSysI2CBus()'s doc
// comment in bus_HAL.h.
bool i2cWriteAddr(uint8_t addr, uint8_t reg, uint8_t value) {
    TwoWire *wire = getSysI2CBus();
    if (wire == nullptr) return false;
    lockSysI2CBus();
    wire->beginTransmission(addr);
    wire->write(reg);
    wire->write(value);
    bool ok = wire->endTransmission() == 0;
    unlockSysI2CBus();
    return ok;
}

bool i2cReadAddr(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    TwoWire *wire = getSysI2CBus();
    if (wire == nullptr) return false;
    lockSysI2CBus();
    wire->beginTransmission(addr);
    wire->write(reg);
    if (wire->endTransmission(false) != 0) {
        unlockSysI2CBus();
        return false; // no repeated-start support -> bail
    }
    size_t got = wire->requestFrom((int)addr, (int)len);
    if (got != len) {
        unlockSysI2CBus();
        return false;
    }
    for (size_t i = 0; i < len; i++) buf[i] = (uint8_t)wire->read();
    unlockSysI2CBus();
    return true;
}

bool i2cWrite(uint8_t reg, uint8_t value) { return i2cWriteAddr(bmi270Addr, reg, value); }

bool i2cRead(uint8_t reg, uint8_t *buf, size_t len) { return i2cReadAddr(bmi270Addr, reg, buf, len); }

// Writes a single <=32-byte chunk starting at `reg` in one I2C transaction (register address
// byte + up to 32 data bytes), used both for the 2-byte INIT_ADDR_0/1 word-address write and
// for each 32-byte page of the INIT_DATA config upload below.
bool i2cWriteChunk(uint8_t reg, const uint8_t *data, size_t len) {
    TwoWire *wire = getSysI2CBus();
    if (wire == nullptr) return false;
    lockSysI2CBus();
    wire->beginTransmission(bmi270Addr);
    wire->write(reg);
    wire->write(data, len);
    bool ok = wire->endTransmission() == 0;
    unlockSysI2CBus();
    return ok;
}

// Uploads the BMI270 config-file blob, mirroring Bosch's own reference driver (bmi2.c's
// upload_file()/write_config_file()) exactly: the chip has NO internal auto-incrementing
// write pointer for INIT_DATA across separate I2C transactions, so every 32-byte page must be
// preceded by writing that page's 16-bit *word* address (index/2, low nibble then high byte)
// to INIT_ADDR_0/INIT_ADDR_1. Streaming the whole blob at INIT_DATA without this address step
// (as an earlier revision did) writes every page to the same location, corrupting the upload.
bool bmi270UploadConfigFile(const uint8_t *data, size_t len) {
    constexpr size_t CHUNK = 32;
    for (size_t off = 0; off < len; off += CHUNK) {
        size_t n = min(CHUNK, len - off);
        uint16_t word = (uint16_t)(off / 2);
        uint8_t addr[2] = {(uint8_t)(word & 0x0F), (uint8_t)(word >> 4)};
        if (!i2cWriteChunk(REG_INIT_ADDR_0, addr, sizeof(addr))) return false;
        if (!i2cWriteChunk(REG_INIT_DATA, data + off, n)) return false;
    }
    return true;
}

} // namespace

bool imu_detect() {
    detected = false;
    initialized = false;

    // Only boards that actually wire a sys_i2c bus can have a BMI270 on it.
    if (bruceConfigPins.sys_i2c.sda < 0 || bruceConfigPins.sys_i2c.scl < 0) {
        Serial.println("DEBUG: IMU - no sys_i2c wired, skipping BMI270 probe");
        return false;
    }

    // Retries + both possible addresses: right after a shared-bus peripheral (e.g. TCA8418)
    // just finished its own init sequence, the very first transaction to a different address
    // can spuriously fail on some ESP32 Wire implementations - a couple of quick retries costs
    // nothing at boot and avoids a false "not present" verdict.
    static const uint8_t addrsToTry[] = {BMI270_I2C_ADDR_LOW, BMI270_I2C_ADDR_HIGH};
    for (uint8_t addr : addrsToTry) {
        for (int attempt = 0; attempt < 3 && !detected; attempt++) {
            uint8_t chipId = 0;
            bool ok = i2cReadAddr(addr, REG_CHIP_ID, &chipId, 1);
            Serial.printf(
                "DEBUG: IMU - probe addr=0x%02X attempt=%d ok=%d chipId=0x%02X\n", addr, attempt, (int)ok, chipId
            );
            if (ok && chipId == BMI270_CHIP_ID) {
                bmi270Addr = addr;
                detected = true;
            } else {
                delay(5);
            }
        }
        if (detected) break;
    }

    Serial.printf("DEBUG: IMU - BMI270 %s\n", detected ? "detected" : "NOT detected");
    return detected;
}

bool imu_available() { return detected; }

bool imu_init() {
    if (!detected) return false;
    if (initialized) return true;

    // Soft-reset so we start from a known state, then give the chip time to come back up.
    i2cWrite(REG_CMD, CMD_SOFTRESET);
    delay(5);

    // Disable advanced power save so register reads/writes are always honored immediately.
    i2cWrite(REG_PWR_CONF, 0x00);
    delayMicroseconds(450);

    // Mandatory BMI270 config-file upload (datasheet 4.4, "Device Initialization"): the chip
    // stays in a non-functional "not_init" state - and every sensor register (including the
    // gyro data used for heading) reads back stale/zero - until this exact 8KB blob has been
    // burst-written to INIT_DATA with INIT_CTRL toggled 0x00 -> upload -> 0x01.
    i2cWrite(REG_INIT_CTRL, 0x00);
    bool uploadOk = bmi270UploadConfigFile(BMI270_CONFIG_FILE, sizeof(BMI270_CONFIG_FILE));
    i2cWrite(REG_INIT_CTRL, 0x01);
    delay(20); // datasheet: wait >=20ms for INTERNAL_STATUS to reflect the upload result

    uint8_t internalStatus = 0;
    bool configOk =
        uploadOk && i2cRead(REG_INTERNAL_STATUS, &internalStatus, 1) && (internalStatus & 0x01) != 0;
    Serial.printf(
        "DEBUG: IMU - config upload %s, INTERNAL_STATUS=0x%02X\n", configOk ? "ok" : "FAILED", internalStatus
    );
    if (!configOk) return false;

    // Power on both the accelerometer and gyroscope (PWR_CTRL = 0x06).
    i2cWrite(REG_PWR_CTRL, 0x06);
    delay(50); // datasheet: gyro startup time is 45ms before valid samples are produced

    // Normal filter/bandwidth, high performance, ODR=100Hz (0xA8); +/-2000 dps range.
    i2cWrite(REG_GYR_CONF, 0xA8);
    i2cWrite(REG_GYR_RANGE, 0x00);

    // Measure resting zero-rate gyro offset across a few samples to prevent static drift.
    float sumGz = 0.0f;
    int samplesTaken = 0;
    for (int i = 0; i < 16; i++) {
        uint8_t raw[6];
        if (i2cRead(REG_GYR_DATA, raw, sizeof(raw))) {
            int16_t gz = (int16_t)((raw[5] << 8) | raw[4]);
            sumGz += (float)gz;
            samplesTaken++;
        }
        delay(2);
    }
    gyroZOffset = (samplesTaken > 0) ? ((sumGz / samplesTaken) * GYRO_DPS_PER_LSB) : 0.0f;

    uint8_t chipId = 0;
    initialized = i2cRead(REG_CHIP_ID, &chipId, 1) && chipId == BMI270_CHIP_ID;
    lastSampleMs = millis();
    return initialized;
}

float imu_get_heading_delta_deg() {
    if (!detected) return 0.0f;
    if (!initialized && !imu_init()) return 0.0f;

    uint8_t raw[6];
    if (!i2cRead(REG_GYR_DATA, raw, sizeof(raw))) return headingDeg;

    int16_t gz = (int16_t)((raw[5] << 8) | raw[4]);

    // Turning clockwise around the Z axis (held flat in hand) yields negative raw gz.
    // Invert and subtract calibrated zero-rate bias so clockwise rotation increases headingDeg.
    float rawDps = gz * GYRO_DPS_PER_LSB;
    float dps = -(rawDps - gyroZOffset);

    // Small deadband to eliminate stationary gyro noise drift while resting still.
    if (fabsf(dps) < 0.4f) dps = 0.0f;

    uint32_t now = millis();
    float dtSec = (lastSampleMs == 0) ? 0.0f : (now - lastSampleMs) / 1000.0f;
    lastSampleMs = now;

    // Clamp dt so a stall (e.g. a slow render frame) can't inject a huge, bogus heading jump.
    if (dtSec > 0.2f) dtSec = 0.2f;

    headingDeg += dps * dtSec;
    headingDeg = fmodf(headingDeg, 360.0f);
    if (headingDeg < 0.0f) headingDeg += 360.0f;

    return headingDeg;
}

void imu_reset_heading() {
    headingDeg = 0.0f;
    lastSampleMs = millis();
}
