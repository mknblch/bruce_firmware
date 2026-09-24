#ifndef BLE_OUI_H
#define BLE_OUI_H

#include <Arduino.h>
#include <NimBLEAddress.h>
#include <NimBLEAdvertisedDevice.h>

#if !defined(LITE_VERSION)

//=============================================================================
// BLE Vendor & OUI Lookup Engine
// Dual-Layer Resolution:
//   Layer 1: High-speed PROGMEM Flash tables for SIG Company IDs & Common OUIs
//   Layer 2: SD card / LittleFS storage search (/ble/oui.bin, /oui.bin, /ChimeraBLE/oui.bin, etc.)
//=============================================================================

/**
 * High-speed, zero-allocation vendor name lookup from PROGMEM Flash table.
 * Thread-safe for use inside NimBLE scan callbacks.
 */
const char *getBleCompanyIdName(uint16_t companyId);
const char *getBleOuiName(uint32_t oui24);
const char *getBleOuiNameFromMacBytes(const uint8_t *macBytes);

/**
 * Resolve vendor name from advertised device packet (SIG Company ID in Mfg Data)
 * or Public MAC address (OUI).
 * 
 * @param dev Pointer to NimBLEAdvertisedDevice
 * @param allowSdLookup Whether to perform SD storage lookup if Flash lookup fails
 * @return Resolved vendor string, or empty string if unknown
 */
String resolveBleVendor(const NimBLEAdvertisedDevice *dev, bool allowSdLookup = false);

/**
 * Resolve vendor name from BLE Address (Public type only).
 * 
 * @param address NimBLEAddress
 * @param allowSdLookup Whether to check SD storage
 * @return Resolved vendor string, or empty string if unknown
 */
String resolveBleOui(const NimBLEAddress &address, bool allowSdLookup = false);

/**
 * Resolve vendor name from 24-bit OUI integer (e.g. 0xAC6784 -> Espressif).
 * 
 * @param oui24 24-bit OUI (byte0 << 16 | byte1 << 8 | byte2)
 * @param allowSdLookup Whether to check SD storage
 * @return Resolved vendor string, or empty string if unknown
 */
String resolveBleOui(uint32_t oui24, bool allowSdLookup = false);

/**
 * Resolve Bluetooth SIG Company Identifier (16-bit).
 * 
 * @param companyId 16-bit company identifier
 * @return Company name, or empty string if unknown
 */
String resolveBleCompanyId(uint16_t companyId);

/**
 * Check if an external OUI database exists on SD card or LittleFS.
 */
bool isSdOuiDatabaseAvailable();

#endif // !LITE_VERSION

#endif // BLE_OUI_H
