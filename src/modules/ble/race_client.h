#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <vector>
#include <functional>

#if !defined(LITE_VERSION)

//=============================================================================
// RACE Protocol Definitions & Constants
//=============================================================================

#define RACE_MAGIC_STD              0x05
#define RACE_MAGIC_EXT              0x15

#define RACE_TYPE_REQ               0x5A    // Host -> Device (Expects Response)
#define RACE_TYPE_REQ_NO_RSP        0x5C    // Host -> Device (No Response)
#define RACE_TYPE_RSP               0x5B    // Device -> Host (Response)
#define RACE_TYPE_IND               0x5D    // Device -> Host (Indication/Notification)

// Known Command Opcodes
#define RACE_CMD_READ_SDK_VERSION   0x0301
#define RACE_CMD_STORAGE_PAGE_PROG  0x0402
#define RACE_CMD_STORAGE_PAGE_READ  0x0403
#define RACE_CMD_PARTITION_ERASE    0x0404
#define RACE_CMD_GET_LINK_KEY       0x0CC0
#define RACE_CMD_GET_BD_ADDRESS     0x0CD5
#define RACE_CMD_READ_ADDRESS       0x1680
#define RACE_CMD_FOTA_PARTITION_INF 0x1C00
#define RACE_CMD_FOTA_INTEGRITY_CHK 0x1C01
#define RACE_CMD_FOTA_COMMIT        0x1C02
#define RACE_CMD_FOTA_STOP          0x1C03
#define RACE_CMD_FOTA_WRITE_STATE   0x1C06
#define RACE_CMD_FOTA_START         0x1C08
#define RACE_CMD_FOTA_START_TRANS   0x1C0A
#define RACE_CMD_GET_BUILD_VERSION  0x1E08

// Known RACE GATT Service & Characteristic UUIDs
#define RACE_UUID_AIROHA_SERVICE    "5052494D-2DAB-0341-6972-6F6861424C45"
#define RACE_UUID_AIROHA_TX         "43484152-2DAB-3241-6972-6F6861424C45"
#define RACE_UUID_AIROHA_RX         "43484152-2DAB-3141-6972-6F6861424C45"
#define RACE_UUID_AIROHA_RX_ALT     "43484152-2DAB-3041-6972-6F6861424C45"

#define RACE_UUID_SONY_SERVICE      "dc405470-a351-4a59-97d8-2e2e3b207fbb"
#define RACE_UUID_SONY_TX           "bfd869fa-a3f2-4c2f-bcff-3eb1ec80cead"
#define RACE_UUID_SONY_RX           "2a6b6575-faf6-418c-923f-ccd63a56d955"

#define RACE_UUID_TRSPX_SERVICE     "49535343-FE7D-4AE5-8FA9-9FAFD205E455"
#define RACE_UUID_TRSPX_TX          "49535343-8841-43F4-A8D4-ECBE34729BB3"
#define RACE_UUID_TRSPX_RX          "49535343-1E4D-4BD9-BA61-23C647249616"

// 16-bit / Short Airoha UART & FOTA profiles
#define RACE_UUID_AIROHA_16BIT_SRV  "0000fef0-0000-1000-8000-00805f9b34fb"
#define RACE_UUID_AIROHA_16BIT_TX   "0000fef1-0000-1000-8000-00805f9b34fb"
#define RACE_UUID_AIROHA_16BIT_RX   "0000fef2-0000-1000-8000-00805f9b34fb"

// Known RACE Bluetooth Classic RFCOMM UUIDs
#define RACE_UUID_RFCOMM_AIROHA_SPP "00000000-0000-0000-0099-aabbccddeeff"
#define RACE_UUID_RFCOMM_SONY       "8901dfa8-5c7e-4d8f-9f0c-c2b70683f5f0"
#define RACE_UUID_RFCOMM_BOSE       "2d064aa9-32b5-4970-865c-643742bd2862"

#pragma pack(push, 1)
struct RaceHeader {
    uint8_t  head;      // 0x05 or 0x15
    uint8_t  type;      // 0x5A (Req), 0x5B (Rsp), etc.
    uint16_t length;    // Payload length + 2 (includes cmdId)
    uint16_t cmdId;     // Command ID
};
#pragma pack(pop)

// Data Structures
struct RaceDeviceInfo {
    String sdkInfo;
    String buildVersion;
    String classicBdAddr;
    int linkKeyCount = 0;
    std::vector<String> linkKeys;
    bool hasRaceService = false;
    String matchedServiceUuid;
};

struct RacePartitionEntry {
    uint32_t address = 0;
    uint32_t length = 0;
    uint8_t  type = 0;
    String   name;
};

enum RaceVulnStatus {
    RACE_VULN_UNKNOWN = 0,
    RACE_VULN_NOT_APPLICABLE,
    RACE_VULN_FIXED,
    RACE_VULN_VULNERABLE
};

struct RaceVulnerabilityReport {
    RaceVulnStatus cve2025_20700 = RACE_VULN_UNKNOWN; // Missing GATT authentication
    RaceVulnStatus cve2025_20701 = RACE_VULN_UNKNOWN; // Missing Classic BR/EDR pairing
    RaceVulnStatus raceOverBle   = RACE_VULN_UNKNOWN; // RACE exposed over BLE
    String details;
};

//=============================================================================
// RaceClient Class
//=============================================================================

class RaceClient {
public:
    RaceClient();
    ~RaceClient();

    // Connection Lifecycle
    bool connect(const NimBLEAddress &address, uint32_t timeoutMs = 8000);
    bool attachClient(NimBLEClient *pClient);
    void disconnect();
    bool isConnected() const;

    // Discovery & Handshake
    bool discoverRaceService();
    const String &getServiceUuid() const { return m_serviceUuid; }

    // Core Protocol Transport
    bool sendRawPacket(const uint8_t *data, size_t len);
    bool sendCommandSync(uint8_t head, uint8_t type, uint16_t cmdId,
                         const uint8_t *payload, size_t payloadLen,
                         std::vector<uint8_t> &response, uint32_t timeoutMs = 3500);

    // High & Moderate Value Operations
    bool probeVulnerability(RaceVulnerabilityReport &report,
                            std::function<void(const String &stepMsg)> progressCb = nullptr);
    bool fetchDeviceInfo(RaceDeviceInfo &info,
                         std::function<void(const String &stepMsg)> progressCb = nullptr);
    bool fetchMediaInfo(String &track, String &album, String &artist, String &genre);
    bool readRamWord(uint32_t address, uint32_t &outWord);
    bool readRam(uint32_t address, size_t length, std::vector<uint8_t> &outData,
                 std::function<void(size_t done, size_t total)> progressCb = nullptr);
    bool readFlashPage(uint32_t address, uint8_t *outPage256, uint8_t storageType = 0);
    bool readFlash(uint32_t address, size_t length, std::vector<uint8_t> &outData,
                   std::function<void(size_t done, size_t total)> progressCb = nullptr);
    bool getPartitionTable(std::vector<RacePartitionEntry> &partitions);
    bool dumpMemoryToStorage(bool isRam, uint32_t address, size_t size,
                             String &outSavedPath,
                             std::function<void(size_t done, size_t total)> progressCb = nullptr);

    // Notification callback handler (called internally by NimBLE)
    void onNotify(NimBLERemoteCharacteristic *pChar, uint8_t *pData, size_t length, bool isNotify);

private:
    NimBLEClient *m_pClient;
    bool m_ownsClient;
    NimBLERemoteService *m_pService;
    NimBLERemoteCharacteristic *m_pTxChar;
    NimBLERemoteCharacteristic *m_pRxChar;
    String m_serviceUuid;

    // Incoming packet reassembly & sync state
    SemaphoreHandle_t m_syncSemaphore;
    SemaphoreHandle_t m_packetMutex;
    std::vector<uint8_t> m_rxBuffer;
    std::vector<uint8_t> m_lastResponse;
    uint16_t m_expectedLength;
    bool m_expectingResponse;
    uint16_t m_expectedCmdId;

    void resetRxState();
};

//=============================================================================
// UI & CLI Entry Points
//=============================================================================

void raceMainMenu();
void launchRaceForDevice(NimBLEClient *pClient, const String &devName, const NimBLEAddress &address);
bool raceCli(const String &macStr, uint8_t addrType, const String &subCmd,
             const String &param1 = "", const String &param2 = "", const String &param3 = "");

#endif // !LITE_VERSION
