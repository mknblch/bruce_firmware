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
constexpr uint8_t REG_ACC_CONF = 0x40;
constexpr uint8_t REG_ACC_RANGE = 0x41;
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

// ACC_RANGE = 0x01 -> +/-4g.
constexpr float ACC_LSB_PER_G = 8192.0f;

// Stationary detection (ZUPT): below this residual rate / |a|-1g deviation for STILL_MS the
// gyro bias is re-estimated on the fly and heading is frozen.
constexpr float STILL_GYRO_DPS = 1.0f;
constexpr float STILL_ACC_G = 0.06f;
constexpr uint32_t STILL_MS = 300;
constexpr float BIAS_ALPHA = 0.02f;
constexpr float RATE_DEADBAND_DPS = 0.1f;

// Step detector / dead reckoning.
constexpr float STEP_LENGTH_M = 0.7f;
constexpr float STEP_ARM_G = 0.12f;
constexpr float STEP_REARM_G = 0.03f;
constexpr uint32_t STEP_MIN_INTERVAL_MS = 300;

constexpr size_t HEADING_HISTORY = 128;

bool detected = false;
bool initialized = false;
float headingDeg = 0.0f;
float gyroBias[3] = {0.0f, 0.0f, 0.0f}; // dps, sensor frame
float gravityVec[3] = {0.0f, 0.0f, 1.0f}; // low-passed accelerometer (g), sensor frame
uint32_t lastSampleMs = 0;
float prevYawRate = 0.0f;
bool stationary = false;
uint32_t stillSinceMs = 0;
uint32_t stepCount = 0;
float posX = 0.0f, posY = 0.0f;
float dynAccLp = 0.0f;
bool stepArmed = true;
uint32_t lastStepMs = 0;

struct HeadingSample {
    uint32_t ms;
    float deg;
};
HeadingSample headingHist[HEADING_HISTORY];
size_t headingHistHead = 0;
size_t headingHistCount = 0;

void pushHeadingSample(uint32_t ms, float deg) {
    headingHist[headingHistHead] = {ms, deg};
    headingHistHead = (headingHistHead + 1) % HEADING_HISTORY;
    if (headingHistCount < HEADING_HISTORY) headingHistCount++;
}

float wrapDeg360(float d) {
    d = fmodf(d, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return d;
}

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
        wire->endTransmission(true);
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

// Uploads the BMI270 config-file blob, mirroring Bosch's own reference driver (bmi2.c's
// upload_file()/write_config_file()) exactly: the chip has NO internal auto-incrementing
// write pointer for INIT_DATA across separate I2C transactions, so every 32-byte page must be
// preceded by writing that page's 16-bit *word* address (index/2, low nibble then high byte)
// to INIT_ADDR_0/INIT_ADDR_1. The entire upload holds lockSysI2CBus() so background keyboard
// polling tasks cannot interleave transactions on the shared I2C bus.
bool bmi270UploadConfigFile(const uint8_t *data, size_t len) {
    constexpr size_t CHUNK = 32;
    TwoWire *wire = getSysI2CBus();
    if (wire == nullptr) return false;

    lockSysI2CBus();
    for (size_t off = 0; off < len; off += CHUNK) {
        size_t n = min(CHUNK, len - off);
        uint16_t word = (uint16_t)(off / 2);
        uint8_t addr[2] = {(uint8_t)(word & 0x0F), (uint8_t)(word >> 4)};

        wire->beginTransmission(bmi270Addr);
        wire->write(REG_INIT_ADDR_0);
        wire->write(addr, sizeof(addr));
        if (wire->endTransmission() != 0) {
            unlockSysI2CBus();
            return false;
        }

        wire->beginTransmission(bmi270Addr);
        wire->write(REG_INIT_DATA);
        wire->write(data + off, n);
        if (wire->endTransmission() != 0) {
            unlockSysI2CBus();
            return false;
        }
    }
    unlockSysI2CBus();
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
    // Accelerometer: 100Hz, normal filter, +/-4g (gravity direction + step detection).
    i2cWrite(REG_ACC_CONF, 0xA8);
    i2cWrite(REG_ACC_RANGE, 0x01);

    // Measure resting zero-rate gyro offset across a few samples to prevent static drift.
    float sum[3] = {0, 0, 0};
    int samplesTaken = 0;
    for (int i = 0; i < 16; i++) {
        uint8_t raw[6];
        if (i2cRead(REG_GYR_DATA, raw, sizeof(raw))) {
            for (int k = 0; k < 3; k++) sum[k] += (float)(int16_t)((raw[2 * k + 1] << 8) | raw[2 * k]);
            samplesTaken++;
        }
        delay(2);
    }
    for (int k = 0; k < 3; k++)
        gyroBias[k] = (samplesTaken > 0) ? ((sum[k] / samplesTaken) * GYRO_DPS_PER_LSB) : 0.0f;

    uint8_t chipId = 0;
    initialized = i2cRead(REG_CHIP_ID, &chipId, 1) && chipId == BMI270_CHIP_ID;
    lastSampleMs = millis();
    return initialized;
}

void imu_update() {
    if (!detected) return;
    if (!initialized && !imu_init()) return;

    // Accelerometer (0x0C..0x11) and gyroscope (0x12..0x17) registers are contiguous.
    uint8_t raw[12];
    if (!i2cRead(REG_ACC_DATA, raw, sizeof(raw))) return;
    auto s16 = [&](int i) { return (float)(int16_t)((raw[i + 1] << 8) | raw[i]); };
    float a[3] = {s16(0) / ACC_LSB_PER_G, s16(2) / ACC_LSB_PER_G, s16(4) / ACC_LSB_PER_G};
    float g[3] = {s16(6) * GYRO_DPS_PER_LSB, s16(8) * GYRO_DPS_PER_LSB, s16(10) * GYRO_DPS_PER_LSB};

    uint32_t now = millis();
    float dtSec = (lastSampleMs == 0) ? 0.0f : (now - lastSampleMs) / 1000.0f;
    lastSampleMs = now;
    // Clamp dt so a stall can't inject a huge, bogus heading jump.
    if (dtSec > 0.2f) dtSec = 0.2f;

    // Gravity direction (low-passed) -> works at any device tilt.
    for (int k = 0; k < 3; k++) gravityVec[k] += 0.05f * (a[k] - gravityVec[k]);
    float gNorm = sqrtf(
        gravityVec[0] * gravityVec[0] + gravityVec[1] * gravityVec[1] + gravityVec[2] * gravityVec[2]
    );
    if (gNorm < 0.2f) gNorm = 1.0f;

    float w[3] = {g[0] - gyroBias[0], g[1] - gyroBias[1], g[2] - gyroBias[2]};
    float aMag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    float wMag = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);

    // Zero-velocity update: when really still, refine the bias and freeze heading.
    bool still = wMag < STILL_GYRO_DPS && fabsf(aMag - 1.0f) < STILL_ACC_G;
    if (!still) {
        stillSinceMs = 0;
        stationary = false;
    } else {
        if (stillSinceMs == 0) stillSinceMs = now;
        if (now - stillSinceMs >= STILL_MS) {
            stationary = true;
            for (int k = 0; k < 3; k++) gyroBias[k] += BIAS_ALPHA * (g[k] - gyroBias[k]);
        }
    }

    // Yaw rate = rotation about the gravity axis; clockwise (seen from above) is positive.
    float yawRate = 0.0f;
    if (!stationary) {
        yawRate = -(w[0] * gravityVec[0] + w[1] * gravityVec[1] + w[2] * gravityVec[2]) / gNorm;
        if (fabsf(yawRate) < RATE_DEADBAND_DPS) yawRate = 0.0f;
    }

    headingDeg = wrapDeg360(headingDeg + 0.5f * (yawRate + prevYawRate) * dtSec);
    prevYawRate = yawRate;
    pushHeadingSample(now, headingDeg);

    // Step detection on the dynamic acceleration magnitude, dead-reckoned along the heading.
    float dyn = fabsf(aMag - 1.0f);
    dynAccLp += 0.3f * (dyn - dynAccLp);
    if (stepArmed && dynAccLp > STEP_ARM_G && (now - lastStepMs) > STEP_MIN_INTERVAL_MS) {
        stepArmed = false;
        lastStepMs = now;
        stepCount++;
        float rad = headingDeg * (M_PI / 180.0f);
        posX += STEP_LENGTH_M * sinf(rad);
        posY += STEP_LENGTH_M * cosf(rad);
    } else if (!stepArmed && dynAccLp < STEP_REARM_G) {
        stepArmed = true;
    }
}

float imu_get_heading_delta_deg() {
    if (!detected) return 0.0f;
    imu_update();
    return headingDeg;
}

float imu_heading_at(uint32_t ms) {
    if (headingHistCount == 0) return headingDeg;
    size_t newest = (headingHistHead + HEADING_HISTORY - 1) % HEADING_HISTORY;
    size_t oldest = (headingHistHead + HEADING_HISTORY - headingHistCount) % HEADING_HISTORY;
    if ((int32_t)(ms - headingHist[newest].ms) >= 0) return headingHist[newest].deg;
    if ((int32_t)(ms - headingHist[oldest].ms) <= 0) return headingHist[oldest].deg;
    for (size_t n = 1; n < headingHistCount; n++) {
        size_t i1 = (newest + HEADING_HISTORY - n + 1) % HEADING_HISTORY;
        size_t i0 = (newest + HEADING_HISTORY - n) % HEADING_HISTORY;
        const HeadingSample &s0 = headingHist[i0];
        const HeadingSample &s1 = headingHist[i1];
        if ((int32_t)(ms - s0.ms) >= 0) {
            uint32_t span = s1.ms - s0.ms;
            if (span == 0) return s1.deg;
            float f = (float)(ms - s0.ms) / (float)span;
            float diff = s1.deg - s0.deg;
            while (diff > 180.0f) diff -= 360.0f;
            while (diff < -180.0f) diff += 360.0f;
            return wrapDeg360(s0.deg + f * diff);
        }
    }
    return headingHist[oldest].deg;
}

uint32_t imu_step_count() { return stepCount; }

void imu_get_position(float &xMeters, float &yMeters) {
    xMeters = posX;
    yMeters = posY;
}

bool imu_is_stationary() { return stationary; }

void imu_reset_heading() {
    headingDeg = 0.0f;
    lastSampleMs = millis();
    prevYawRate = 0.0f;
    stillSinceMs = 0;
    stationary = false;
    stepCount = 0;
    posX = posY = 0.0f;
    dynAccLp = 0.0f;
    stepArmed = true;
    lastStepMs = 0;
    headingHistHead = 0;
    headingHistCount = 0;
}

bool imu_calibrate(uint32_t durationMs, std::function<void(int)> progressCb) {
    if (!detected) return false;
    if (!initialized && !imu_init()) return false;

    constexpr uint32_t sampleIntervalMs = 10;
    uint32_t totalSamples = durationMs / sampleIntervalMs;
    if (totalSamples < 10) totalSamples = 10;

    float sum[3] = {0, 0, 0};
    float sumA[3] = {0, 0, 0};
    int samplesTaken = 0;
    int lastPct = -1;

    for (uint32_t i = 0; i < totalSamples; i++) {
        uint8_t raw[12];
        if (i2cRead(REG_ACC_DATA, raw, sizeof(raw))) {
            for (int k = 0; k < 3; k++) {
                sumA[k] += (float)(int16_t)((raw[2 * k + 1] << 8) | raw[2 * k]) / ACC_LSB_PER_G;
                sum[k] += (float)(int16_t)((raw[6 + 2 * k + 1] << 8) | raw[6 + 2 * k]);
            }
            samplesTaken++;
        }
        if (progressCb) {
            int pct = (int)((i + 1) * 100 / totalSamples);
            if (pct != lastPct && (pct % 5 == 0 || pct == 100)) {
                lastPct = pct;
                progressCb(pct);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(sampleIntervalMs));
    }

    if (samplesTaken > 0) {
        for (int k = 0; k < 3; k++) {
            gyroBias[k] = (sum[k] / (float)samplesTaken) * GYRO_DPS_PER_LSB;
            gravityVec[k] = sumA[k] / (float)samplesTaken;
        }
        Serial.printf(
            "DEBUG: IMU - gyro bias=%.3f/%.3f/%.3f dps (%d samples)\n",
            gyroBias[0], gyroBias[1], gyroBias[2], samplesTaken
        );
    }
    imu_reset_heading();
    return (samplesTaken >= (int)(totalSamples / 2));
}
