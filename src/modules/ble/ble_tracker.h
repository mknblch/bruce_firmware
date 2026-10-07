#ifndef __BLE_TRACKER_H__
#define __BLE_TRACKER_H__

#include <globals.h>
#include <stdint.h>

/*
 * BLE Tracker: passively locates a chosen BLE device by listening to its advertisements
 * only (no scan requests, no connection). See BleTrackerMenu() for the entry point wired
 * into BleMenu.
 */

// Which radio produced a BleTrackerSample. ESP32 (the internal NimBLE stack) is the sole
// scanning device the tracker uses - see BleMenu.cpp/history for why a user-selectable or
// NRF24-assisted scanner was tried and then dropped again (no calibrated RSSI on the RF24
// driver Bruce uses, so it never carried genuine range information).
enum BleTrackerSampleSource : uint8_t { TRACK_SRC_ESP32 = 0 };

// One RSSI sighting of the locked target. headingBucket is 0..35 (36 x 10 deg sectors) on IMU
// boards and always 0 otherwise.
struct BleTrackerSample {
    int8_t rssi;
    uint16_t headingBucket;
    uint32_t timestamp;
    BleTrackerSampleSource source;
};

// Entry point registered in BleMenu: shows saved favorites (if any) plus a "Live Scan"
// option, then hands off to bleTrackerLockTarget() once a device is picked.
void BleTrackerMenu();

// Runs a one-shot passive NimBLE scan (setActiveScan(false)) and renders the results as a
// pick-list, mirroring ble_scan()'s options/loopOptions() flow in ble_common.cpp.
void bleTrackerScanAndPick();

// Locks onto `mac` and offers to save it as a favorite before entering the tracking view.
// `label` is only used for on-screen display and as the default favorite name.
void bleTrackerLockTarget(const String &label, const String &mac, uint8_t addrType = 0xFF);

class NimBLEClient;

// Main tracking loop/UI for a locked target. A supplied address type distinguishes matching
// public/random addresses; 0xFF preserves MAC-only matching for older saved favorites. When
// supplied a connected client or server connection handle, tracking uses link RSSI without
// scanning or changing the connection. Returns when the user presses EscPress.
void bleTrackerRun(const String &targetMac, const String &label, NimBLEClient *pClient = nullptr, uint8_t targetAddrType = 0xFF,
                   uint16_t connectedServerHandle = 0xFFFF);

#endif
