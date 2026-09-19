#ifndef __NRF_BLE_CORE_H
#define __NRF_BLE_CORE_H

#include <Arduino.h>
#include <RF24.h>
#include "modules/NRF24/nrf_common.h"

// BLE Advertising Channels
#define NRF_BLE_CH37 37
#define NRF_BLE_CH38 38
#define NRF_BLE_CH39 39

// nRF24 Physical RF Channels corresponding to BLE Advertising channels
#define NRF_BLE_RF_CH37 2
#define NRF_BLE_RF_CH38 26
#define NRF_BLE_RF_CH39 80

// PDU Types
#define NRF_BLE_ADV_IND         0x00
#define NRF_BLE_ADV_DIRECT_IND  0x01
#define NRF_BLE_ADV_NONCONN_IND 0x02
#define NRF_BLE_SCAN_REQ        0x03
#define NRF_BLE_SCAN_RSP        0x04
#define NRF_BLE_CONNECT_IND     0x05
#define NRF_BLE_ADV_SCAN_IND    0x06

// BLE Packet structure
struct NrfBlePacket {
    uint8_t raw[32];
    uint8_t len;
    uint8_t channel;      // BLE channel: 37, 38, or 39
    uint8_t rf_channel;   // nRF24 RF channel: 2, 26, or 80
    uint8_t pdu_type;
    bool tx_add;          // 0 = Public, 1 = Random
    bool rx_add;
    uint8_t mac[6];
    String mac_str;
    uint8_t adv_data[24];
    uint8_t adv_len;
    bool crc_ok;
    uint32_t calc_crc;
    uint32_t packet_crc;
    String name;
    String vendor;
    uint16_t company_id;
    unsigned long timestamp;
};

// Radio & Physical Layer
bool nrf_ble_init_radio();
void nrf_ble_deinit_radio();
void nrf_ble_set_power(rf24_pa_dbm_e level);
uint8_t nrf_ble_chan_to_rf(uint8_t ble_chan);
uint8_t nrf_ble_rf_to_chan(uint8_t rf_chan);

// Math & Scrambling Primitives
uint8_t nrf_ble_swapbits(uint8_t b);
void nrf_ble_crc(const uint8_t *data, uint8_t len, uint8_t *dst);
void nrf_ble_whiten(uint8_t *data, uint8_t len, uint8_t whitenCoeff);
uint8_t nrf_ble_whiten_start(uint8_t chan);

// Packet Building & Transmission
bool nrf_ble_build_packet(uint8_t *out_buf, uint8_t &out_len, uint8_t pdu_type,
                          const uint8_t *mac, const uint8_t *adv_payload,
                          uint8_t adv_len, uint8_t ble_chan);
bool nrf_ble_send_raw(const uint8_t *packet, uint8_t len, uint8_t ble_chan);
bool nrf_ble_send_adv(uint8_t pdu_type, const uint8_t *mac,
                      const uint8_t *adv_payload, uint8_t adv_len,
                      uint8_t ble_chan = 0xFF);

// Packet Parsing & Analysis
bool nrf_ble_parse_packet(const uint8_t *raw_32, uint8_t ble_chan, NrfBlePacket &pkt);
String nrf_ble_parse_name(const uint8_t *adv_data, uint8_t adv_len);
String nrf_ble_parse_vendor(const uint8_t *adv_data, uint8_t adv_len, uint16_t &company_id);
String nrf_ble_pdu_type_str(uint8_t pdu_type);
void nrf_ble_random_mac(uint8_t *mac);
String nrf_ble_mac_to_str(const uint8_t *mac);

// Preset Packet Builders
uint8_t nrf_ble_build_ibeacon(uint8_t *buf, const uint8_t *uuid, uint16_t major, uint16_t minor, int8_t tx_power);
uint8_t nrf_ble_build_eddystone_url(uint8_t *buf, const String &url, int8_t tx_power);
uint8_t nrf_ble_build_eddystone_uid(uint8_t *buf, const uint8_t *nid, const uint8_t *bid, int8_t tx_power);
uint8_t nrf_ble_build_altbeacon(uint8_t *buf, const uint8_t *beacon_id, uint16_t mfg_id, int8_t ref_rssi);
uint8_t nrf_ble_build_bruce_beacon(uint8_t *buf, const String &name);

// Notification Builders
uint8_t nrf_ble_build_apple_notification(uint8_t *buf, uint8_t notification_type);
uint8_t nrf_ble_build_google_fastpair(uint8_t *buf, uint32_t model_id);
uint8_t nrf_ble_build_samsung_setup(uint8_t *buf, uint8_t setup_type);
uint8_t nrf_ble_build_swift_pair(uint8_t *buf, const String &device_name);

#endif // __NRF_BLE_CORE_H
