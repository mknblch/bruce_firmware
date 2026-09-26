#if !defined(LITE_VERSION)
#include "LoRaRadio.h"
#include "core/bus_HAL.h"
#include "core/configPins.h"
#include "core/display.h"
#include <Arduino.h>

extern BruceConfigPins bruceConfigPins;

volatile bool gLoraPacketReceived = false;
volatile bool gLoraInterruptEnabled = true;

static SPIClass *gLoraSpi = nullptr;
static Module *gLoraModule = nullptr;
static SX1276 *gLora1276 = nullptr;
static SX1262 *gLora1262 = nullptr;
static bool gLoraInitialized = false;
static LoRaRadioType gActiveRadioType = LoRaRadioType::SX1276;
static float gCurFreq = 0.0f;
static float gCurBw = 0.0f;
static uint8_t gCurSf = 0;

bool __attribute__((weak)) prepareBoardLoRaRadio() { return true; }

int getLoraIrqPin() {
#ifdef LORA_IRQ
    return LORA_IRQ;
#else
    return bruceConfigPins.LoRa_bus.io2;
#endif
}

int getLoraBusyPin() {
#ifdef LORA_BUSY
    return LORA_BUSY;
#else
    return bruceConfigPins.LoRa_bus.io1;
#endif
}

int getLoraResetPin() { return bruceConfigPins.LoRa_bus.io0; }
int getLoraCsPin() { return bruceConfigPins.LoRa_bus.cs; }

static void IRAM_ATTR onLoraPacketInterrupt() {
    if (!gLoraInterruptEnabled) return;
    gLoraPacketReceived = true;
}

static int startLoRaReceiveOnActiveRadio() {
    if (gActiveRadioType == LoRaRadioType::SX1262 && gLora1262) {
        const uint32_t irqFlags = RADIOLIB_SX126X_IRQ_RX_DONE | RADIOLIB_SX126X_IRQ_TIMEOUT |
                                  RADIOLIB_SX126X_IRQ_CRC_ERR | RADIOLIB_SX126X_IRQ_HEADER_ERR |
                                  RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED | RADIOLIB_SX126X_IRQ_SYNC_WORD_VALID |
                                  RADIOLIB_SX126X_IRQ_HEADER_VALID;
        const uint32_t irqMask = RADIOLIB_SX126X_IRQ_RX_DONE | RADIOLIB_SX126X_IRQ_TIMEOUT |
                                 RADIOLIB_SX126X_IRQ_CRC_ERR | RADIOLIB_SX126X_IRQ_HEADER_ERR;
        return gLora1262->startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, irqFlags, irqMask, 0);
    }
    if (gLora1276) return gLora1276->startReceive();
    return -1;
}

static SPIClass *selectLoraSPI() {
    SPIClass *bus = acquireSPIBus(
        bruceConfigPins.LoRa_bus.sck, bruceConfigPins.LoRa_bus.miso, bruceConfigPins.LoRa_bus.mosi
    );
    if (!bus) {
        Serial.println("[LoRa] No hardware SPI bus available, using default SPI");
        return &SPI;
    }
    return bus;
}

bool isLoraHardwareConfigured() {
    if (getLoraCsPin() == GPIO_NUM_NC || bruceConfigPins.LoRa_bus.mosi == GPIO_NUM_NC ||
        bruceConfigPins.LoRa_bus.miso == GPIO_NUM_NC || bruceConfigPins.LoRa_bus.sck == GPIO_NUM_NC) {
        return false;
    }
    if (getLoraIrqPin() == GPIO_NUM_NC) {
        return false;
    }
    return true;
}

void stopLoRaRadio() {
    gLoraInterruptEnabled = false;
    gLoraPacketReceived = false;
    gLoraInitialized = false;
    gCurFreq = 0.0f;
    gCurBw = 0.0f;
    gCurSf = 0;

    if (gLora1276) {
        gLora1276->standby();
        delete gLora1276;
        gLora1276 = nullptr;
    }
    if (gLora1262) {
        gLora1262->standby();
        delete gLora1262;
        gLora1262 = nullptr;
    }
    if (gLoraModule) {
        delete gLoraModule;
        gLoraModule = nullptr;
    }
    gLoraSpi = nullptr;
}

bool isLoraModulePresent(bool verbose) {
    if (!isLoraHardwareConfigured()) {
        if (verbose) displayError("LoRa pins not configured!", true);
        return false;
    }

    if (gLoraInitialized && (gLora1276 || gLora1262)) {
        return true;
    }

    loadLoRaConfig();
    bool ok = initLoRaRadio(loraConfig, false);
    if (!ok && verbose) {
        displayError("LoRa module not responding!", true);
    }
    return ok;
}

bool initLoRaRadio(const LoRaConfigData &cfg, bool rxMode) {
    gLoraInterruptEnabled = false;
    gLoraPacketReceived = false;
    gLoraInitialized = false;

    if (!isLoraHardwareConfigured()) {
        Serial.println("[LoRa] Pins not configured");
        return false;
    }

    if (!prepareBoardLoRaRadio()) {
        Serial.println("[LoRa] Failed to prepare board LoRa frontend");
        return false;
    }

    stopLoRaRadio();

    gLoraSpi = selectLoraSPI();
    gActiveRadioType = cfg.radioType;

    const int irqPin = getLoraIrqPin();
    const int busyPin = (gActiveRadioType == LoRaRadioType::SX1262) ? getLoraBusyPin() : GPIO_NUM_NC;
    if (gActiveRadioType == LoRaRadioType::SX1262 && busyPin == GPIO_NUM_NC) {
        Serial.println("[LoRa] SX1262 BUSY pin unset; RadioLib will use its 50 ms fallback");
    }
    gLoraModule = new Module(getLoraCsPin(), irqPin, getLoraResetPin(), busyPin, *gLoraSpi);

    int state = RADIOLIB_ERR_NONE;
    if (gActiveRadioType == LoRaRadioType::SX1276) {
        gLora1276 = new SX1276(gLoraModule);
        state = gLora1276->begin(cfg.freqMHz);
        if (state == RADIOLIB_ERR_NONE) state = gLora1276->setSpreadingFactor(cfg.sf);
        if (state == RADIOLIB_ERR_NONE) state = gLora1276->setBandwidth(cfg.bwKHz);
        if (state == RADIOLIB_ERR_NONE) state = gLora1276->setCodingRate(cfg.cr);
        if (state == RADIOLIB_ERR_NONE) state = gLora1276->setSyncWord(cfg.syncWord);
        if (state == RADIOLIB_ERR_NONE) state = gLora1276->setPreambleLength(cfg.preambleLen);
        if (state == RADIOLIB_ERR_NONE) state = gLora1276->setOutputPower(cfg.powerDbm);
        if (state == RADIOLIB_ERR_NONE) state = gLora1276->setCRC(true);
        if (state == RADIOLIB_ERR_NONE) state = gLora1276->explicitHeader();
        if (state == RADIOLIB_ERR_NONE && rxMode) {
            gLora1276->setDio0Action(onLoraPacketInterrupt, RISING);
            state = gLora1276->startReceive();
        }
    } else {
        gLora1262 = new SX1262(gLoraModule);
        state = gLora1262->begin(cfg.freqMHz);
        if (state == RADIOLIB_ERR_NONE) state = gLora1262->setSpreadingFactor(cfg.sf);
        if (state == RADIOLIB_ERR_NONE) state = gLora1262->setBandwidth(cfg.bwKHz);
        if (state == RADIOLIB_ERR_NONE) state = gLora1262->setCodingRate(cfg.cr);
        if (state == RADIOLIB_ERR_NONE) state = gLora1262->setSyncWord(cfg.syncWord);
        if (state == RADIOLIB_ERR_NONE) state = gLora1262->setPreambleLength(cfg.preambleLen);
        if (state == RADIOLIB_ERR_NONE) state = gLora1262->setOutputPower(cfg.powerDbm);
        if (state == RADIOLIB_ERR_NONE) state = gLora1262->setCRC(true);
        if (state == RADIOLIB_ERR_NONE) state = gLora1262->explicitHeader();
        if (state == RADIOLIB_ERR_NONE && rxMode) {
            gLora1262->setDio1Action(onLoraPacketInterrupt);
            state = startLoRaReceiveOnActiveRadio();
        }
    }

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] Initialization failed with error code: %d\n", state);
        stopLoRaRadio();
        return false;
    }

    gLoraInitialized = true;
    gLoraInterruptEnabled = true;
    gCurFreq = cfg.freqMHz;
    gCurSf = cfg.sf;
    gCurBw = cfg.bwKHz;
    Serial.printf(
        "[LoRa] Radio init OK: %.3f MHz, SF%d, BW%.1fkHz, CR4/%d, Sync 0x%02X\n",
        cfg.freqMHz, cfg.sf, cfg.bwKHz, cfg.cr, cfg.syncWord
    );
    return true;
}

bool setLoRaFrequency(float freqMHz) {
    if (!gLoraInitialized) return false;
    if (fabs(gCurFreq - freqMHz) < 0.0001f) return true;
    gLoraInterruptEnabled = false;
    gLoraPacketReceived = false;
    int state = RADIOLIB_ERR_NONE;
    if (gActiveRadioType == LoRaRadioType::SX1276 && gLora1276) {
        state = gLora1276->setFrequency(freqMHz);
    } else if (gLora1262) {
        state = gLora1262->setFrequency(freqMHz, true);
    }
    if (state == RADIOLIB_ERR_NONE) gCurFreq = freqMHz;
    gLoraPacketReceived = false;
    gLoraInterruptEnabled = true;
    return (state == RADIOLIB_ERR_NONE);
}

bool setLoRaBandwidth(float bwKHz) {
    if (!gLoraInitialized) return false;
    if (fabs(gCurBw - bwKHz) < 0.01f) return true;
    gLoraInterruptEnabled = false;
    gLoraPacketReceived = false;
    int state = RADIOLIB_ERR_NONE;
    if (gActiveRadioType == LoRaRadioType::SX1276 && gLora1276) {
        state = gLora1276->setBandwidth(bwKHz);
    } else if (gLora1262) {
        state = gLora1262->setBandwidth(bwKHz);
    }
    if (state == RADIOLIB_ERR_NONE) gCurBw = bwKHz;
    gLoraPacketReceived = false;
    gLoraInterruptEnabled = true;
    return (state == RADIOLIB_ERR_NONE);
}

bool setLoRaSpreadingFactor(uint8_t sf) {
    if (!gLoraInitialized) return false;
    if (gCurSf == sf) return true;
    gLoraInterruptEnabled = false;
    gLoraPacketReceived = false;
    int state = RADIOLIB_ERR_NONE;
    if (gActiveRadioType == LoRaRadioType::SX1276 && gLora1276) {
        state = gLora1276->setSpreadingFactor(sf);
    } else if (gLora1262) {
        state = gLora1262->setSpreadingFactor(sf);
    }
    if (state == RADIOLIB_ERR_NONE) gCurSf = sf;
    gLoraPacketReceived = false;
    gLoraInterruptEnabled = true;
    return (state == RADIOLIB_ERR_NONE);
}

bool setLoRaSyncWord(uint8_t syncWord) {
    if (!gLoraInitialized) return false;
    gLoraInterruptEnabled = false;
    gLoraPacketReceived = false;
    int state = RADIOLIB_ERR_NONE;
    if (gActiveRadioType == LoRaRadioType::SX1276 && gLora1276) {
        state = gLora1276->setSyncWord(syncWord);
    } else if (gLora1262) {
        state = gLora1262->setSyncWord(syncWord);
    }
    gLoraPacketReceived = false;
    gLoraInterruptEnabled = true;
    return (state == RADIOLIB_ERR_NONE);
}

bool setLoRaCodingRate(uint8_t cr) {
    if (!gLoraInitialized) return false;
    gLoraInterruptEnabled = false;
    gLoraPacketReceived = false;
    int state = RADIOLIB_ERR_NONE;
    if (gActiveRadioType == LoRaRadioType::SX1276 && gLora1276) {
        state = gLora1276->setCodingRate(cr);
    } else if (gLora1262) {
        state = gLora1262->setCodingRate(cr);
    }
    gLoraPacketReceived = false;
    gLoraInterruptEnabled = true;
    return (state == RADIOLIB_ERR_NONE);
}

bool startLoRaReceive() {
    if (!gLoraInitialized) return false;
    gLoraInterruptEnabled = false;
    gLoraPacketReceived = false;
    int state = RADIOLIB_ERR_NONE;
    if (gActiveRadioType == LoRaRadioType::SX1276 && gLora1276) {
        state = gLora1276->startReceive();
    } else {
        state = startLoRaReceiveOnActiveRadio();
    }
    gLoraPacketReceived = false;
    gLoraInterruptEnabled = true;
    return (state == RADIOLIB_ERR_NONE);
}

bool checkLoRaPacketAvailable() {
    if (!gLoraInitialized) return false;
    return gLoraPacketReceived;
}

int readLoRaRawData(
    uint8_t *buffer, size_t maxLen, float &rssi, float &snr, float &freqErr, size_t &packetLen
) {
    packetLen = 0;
    if (!gLoraInitialized) return -1;
    if (!checkLoRaPacketAvailable()) return -1;

    gLoraInterruptEnabled = false;
    gLoraPacketReceived = false;

    size_t len = 0;
    if (gActiveRadioType == LoRaRadioType::SX1276 && gLora1276) {
        len = gLora1276->getPacketLength();
    } else if (gLora1262) {
        len = gLora1262->getPacketLength();
    }

    if (len == 0) {
        bool headerError = false;
        if (gActiveRadioType == LoRaRadioType::SX1262 && gLora1262) {
            headerError = (gLora1262->getIrqFlags() & RADIOLIB_SX126X_IRQ_HEADER_ERR) != 0;
        }
        startLoRaReceiveOnActiveRadio();
        gLoraPacketReceived = false;
        gLoraInterruptEnabled = true;
        return headerError ? RADIOLIB_ERR_CRC_MISMATCH : -1;
    }

    if (len > maxLen) len = maxLen;

    int state = (gActiveRadioType == LoRaRadioType::SX1262 && gLora1262)
                    ? gLora1262->readData(buffer, len)
                    : (gLora1276 ? gLora1276->readData(buffer, len) : -1);

    if (state == RADIOLIB_ERR_NONE || state == RADIOLIB_ERR_CRC_MISMATCH) {
        packetLen = len;
        if (gActiveRadioType == LoRaRadioType::SX1276 && gLora1276) {
            rssi = gLora1276->getRSSI();
            snr = gLora1276->getSNR();
            freqErr = gLora1276->getFrequencyError();
        } else if (gLora1262) {
            rssi = gLora1262->getRSSI();
            snr = gLora1262->getSNR();
            freqErr = gLora1262->getFrequencyError();
        }
    }

    startLoRaReceiveOnActiveRadio();

    gLoraPacketReceived = false;
    gLoraInterruptEnabled = true;
    return state;
}

bool transmitLoRaRawData(const uint8_t *buffer, size_t len) {
    if (!gLoraInitialized) return false;
    gLoraInterruptEnabled = false;

    int state = (gActiveRadioType == LoRaRadioType::SX1262 && gLora1262)
                    ? gLora1262->transmit((uint8_t *)buffer, len)
                    : (gLora1276 ? gLora1276->transmit((uint8_t *)buffer, len) : -1);

    startLoRaReceiveOnActiveRadio();

    gLoraInterruptEnabled = true;
    return (state == RADIOLIB_ERR_NONE);
}

bool transmitLoRaString(const String &str) {
    return transmitLoRaRawData((const uint8_t *)str.c_str(), str.length());
}

int scanLoRaCAD() {
    if (!gLoraInitialized) return -1;
    gLoraInterruptEnabled = false;
    int state = (gActiveRadioType == LoRaRadioType::SX1262 && gLora1262)
                    ? gLora1262->scanChannel()
                    : (gLora1276 ? gLora1276->scanChannel() : -1);
    gLoraInterruptEnabled = true;
    return state;
}

uint32_t getLoRaIrqFlags() {
    if (!gLoraInitialized) return 0;
    if (gActiveRadioType == LoRaRadioType::SX1262 && gLora1262) {
        return gLora1262->getIrqFlags();
    } else if (gLora1276) {
        return gLora1276->getIrqFlags();
    }
    return 0;
}

float getLoRaInstantRSSI() {
    if (!gLoraInitialized) return -140.0f;
    if (gActiveRadioType == LoRaRadioType::SX1276 && gLora1276) {
        return gLora1276->getRSSI(false);
    } else if (gLora1262) {
        return gLora1262->getRSSI(false);
    }
    return -140.0f;
}

float getLoRaTimeOnAir(size_t len) {
    if (!gLoraInitialized) return 0.0f;
    if (gActiveRadioType == LoRaRadioType::SX1276 && gLora1276) {
        return (float)gLora1276->getTimeOnAir(len) / 1000.0f;
    } else if (gLora1262) {
        return (float)gLora1262->getTimeOnAir(len) / 1000.0f;
    }
    return 0.0f;
}

#endif // !LITE_VERSION
