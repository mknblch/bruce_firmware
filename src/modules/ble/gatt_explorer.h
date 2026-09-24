#ifndef GATT_EXPLORER_H
#define GATT_EXPLORER_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <functional>
#include <vector>

#if !defined(LITE_VERSION)

void gattExplorerMenu();
bool gattConnectCli(const String &macStr, uint8_t addrType = 0);
void gattScanCli(int timeoutSec = 5);

bool gattConnectWithStrategies(const NimBLEAddress &target, NimBLEClient **outClient, int *outError = nullptr, bool *outUserCancelled = nullptr);
String gattFitText(const String &src, int maxPx);
void gattDrawRssi(int x, int y, int rssi, uint16_t color);
int gattListLoop(
    const char *title, int count, const String &hint,
    std::function<void(int idx, int x, int y, int w, bool selected)> drawRow, int *cursor = nullptr
);

// Runs the GATT Explorer's live scan + picker screen (RSSI bars, device count, cancelable
// with ESC/SEL) and, when the user selects a device, returns true and outputs its name/MAC/
// RSSI/address type. Completely unwinds the scanner UI and stack before returning.
// Reuses the exact same scan engine and g_gattSettings (minRSSI, connectable/address-type
// filters, etc.) as the main GATT Explorer menu.
bool gattScanAndPick(String &outName, String &outMac, int &outRssi, uint8_t &outAddrType);

// Settings screen for the shared scan engine (minRSSI, scan timeout, address-type filter,
// etc.) - exposed so other features reusing gattScanAndPick() can offer the same tuning menu.
void gattSettingsMenu();

#endif // !LITE_VERSION

#endif // GATT_EXPLORER_H
