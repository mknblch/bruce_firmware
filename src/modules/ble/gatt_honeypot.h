#ifndef GATT_HONEYPOT_H
#define GATT_HONEYPOT_H

#include <Arduino.h>

#if !defined(LITE_VERSION)

void gattHoneypotMenu();
void runGattHoneypot(const String &jsonFilePath = "");
bool startGattHoneypotService(const String &jsonConfigOrPath = "");
void stopGattHoneypotService();
bool isGattHoneypotActive();
String getGattHoneypotStatus();
String getGattHoneypotLogs();
bool setGattHoneypotAdvertising(bool enabled);

#endif // !LITE_VERSION

#endif // GATT_HONEYPOT_H
