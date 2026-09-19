#ifndef __NRF_BLE_H
#define __NRF_BLE_H

#include <Arduino.h>
#include "nrf_ble_core.h"

// Main entry point for NRF24BLE submenu in Bruce NRF24 menu
void nrf_ble_menu();

// Submodule entry points
void nrf_ble_scanner();
void nrf_ble_beacon_menu();
void nrf_ble_sniffer();
void nrf_ble_notification_menu();

#endif // __NRF_BLE_H
