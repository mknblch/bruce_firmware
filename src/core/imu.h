#pragma once

/*
 * Minimal, board-agnostic BMI270 IMU driver.
 *
 * Presence is runtime-detected (mirrors the UseTCA8418 pattern in
 * boards/m5stack-cardputer/interface.cpp): on boards that wire a BMI270 to
 * bruceConfigPins.sys_i2c (currently only the Cardputer ADV), imu_detect() probes the
 * chip's CHIP_ID register once at startup and caches the result. Boards without a BMI270
 * simply get "not present" back from every call below - this header/driver is a safe no-op
 * everywhere else.
 *
 * Only the handful of registers needed for a coarse heading estimate are touched (CHIP_ID,
 * INIT_CTRL/INIT_DATA/INTERNAL_STATUS, PWR_CTRL/PWR_CONF, GYR_CONF/GYR_RANGE, GYR_DATA). Per
 * the BMI270 datasheet ("Power-On-Reset and Device Initialization"), the chip stays in a
 * non-functional state - every sensor register, including the gyro data this driver reads,
 * comes back stale/zero - until Bosch's ~8KB binary config blob (imu_bmi270_config.h,
 * verbatim from their public BMI270_SensorAPI repo) has been uploaded once after reset. This
 * driver performs that upload in imu_init() before enabling the gyroscope; if the upload
 * fails, imu_init() returns false and imu_get_heading_delta_deg() degrades gracefully to 0
 * instead of hanging or throwing.
 */

#include <stdint.h>

// Probes the BMI270 CHIP_ID register on bruceConfigPins.sys_i2c and caches the result.
// Safe to call even on boards with no sys_i2c wired (returns false immediately). Should be
// called once during startup, right after board-specific sys_i2c pins are finalized (i.e.
// after _post_setup_gpio()).
bool imu_detect();

// Returns the cached result of the last imu_detect() call. Never blocks or touches the bus.
bool imu_available();

// Configures the gyroscope (range/ODR) and powers it on. No-op (returns false) if
// imu_available() is false. Safe to call multiple times.
bool imu_init();

// Integrates the gyroscope's Z axis since the previous call and returns the heading delta in
// degrees, wrapped to [0, 360). Returns 0 on boards without an IMU or if a read fails, so
// callers never need to special-case the no-IMU path beyond checking imu_available() once.
float imu_get_heading_delta_deg();

// Resets the internal heading accumulator to 0. Call this whenever a new tracking session
// starts so stale heading data from a previous session/target doesn't bleed into a new one.
void imu_reset_heading();
