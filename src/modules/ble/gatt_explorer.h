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

#endif // !LITE_VERSION

#endif // GATT_EXPLORER_H
