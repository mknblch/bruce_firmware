#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include <math.h>
#include <string.h>

namespace {
constexpr byte kWriteBurst = 0x40;
constexpr byte kReadSingle = 0x80;
constexpr byte kReadBurst = 0xC0;
constexpr byte kRxFifoMask = 0x7F;
constexpr byte kMaxFifoPayload = 61;
constexpr float kXtalMHz = 26.0f;

const uint8_t pa315[] = {0x12, 0x0D, 0x1C, 0x34, 0x51, 0x85, 0xCB, 0xC2};
const uint8_t pa433[] = {0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0};
const uint8_t pa868[] = {0x03, 0x17, 0x1D, 0x26, 0x37, 0x50, 0x86, 0xCD, 0xC5, 0xC0};
const uint8_t pa915[] = {0x03, 0x0E, 0x1E, 0x27, 0x38, 0x8E, 0x84, 0xCC, 0xC3, 0xC0};

int powerIndex(int power, int count) {
    static const int levels[] = {-30, -20, -15, -10, -6, 0, 5, 7, 10, 12};
    int index = 0;
    while (index + 1 < count && power > levels[index]) index++;
    return index;
}
}

CC1101Driver ELECHOUSE_cc1101;

bool CC1101Driver::isFrequencySupported(float mhz) {
    return isfinite(mhz) && ((mhz >= 300.0f && mhz <= 348.0f) ||
                             (mhz >= 387.0f && mhz <= 464.0f) ||
                             (mhz >= 779.0f && mhz <= 928.0f));
}

bool CC1101Driver::validModule(byte module) const { return module < 6; }

void CC1101Driver::setBeginEndLogic(bool state) { _beginEndLogic = state; }
bool CC1101Driver::getBeginEndLogic() { return _beginEndLogic; }

void CC1101Driver::setSPIinstance(SPIClass *spi) { _spi = spi; }

SPIClass *CC1101Driver::getSPIinstance() {
    if (!_spi) _spi = &_defaultSpi;
    return _spi;
}

void CC1101Driver::setSpiPin(byte sck, byte miso, byte mosi, byte ss) {
    _sck = sck;
    _miso = miso;
    _mosi = mosi;
    _ss = ss;
}

void CC1101Driver::addSpiPin(byte sck, byte miso, byte mosi, byte ss, byte module) {
    if (!validModule(module)) return;
    _hasModulePins = true;
    _sckModules[module] = sck;
    _misoModules[module] = miso;
    _mosiModules[module] = mosi;
    _ssModules[module] = ss;
}

void CC1101Driver::setGDO(byte gdo0, byte gdo2) {
    _gdo0 = gdo0;
    _gdo2 = gdo2;
    if (_gdo0 != 255) pinMode(_gdo0, OUTPUT);
    if (_gdo2 != 255) pinMode(_gdo2, INPUT);
}

void CC1101Driver::setGDO0(byte gdo0) {
    _gdo0 = gdo0;
    if (_gdo0 != 255) pinMode(_gdo0, INPUT);
}

void CC1101Driver::addGDO(byte gdo0, byte gdo2, byte module) {
    if (!validModule(module)) return;
    _hasGdo2Modules = true;
    _gdo0Modules[module] = gdo0;
    _gdo2Modules[module] = gdo2;
}

void CC1101Driver::addGDO0(byte gdo0, byte module) {
    if (!validModule(module)) return;
    _hasGdo0Modules = true;
    _gdo0Modules[module] = gdo0;
}

void CC1101Driver::setModul(byte module) {
    if (!validModule(module) || !_hasModulePins) return;
    if (_transactionActive) endTransaction();
    _module = module;
    _sck = _sckModules[module];
    _miso = _misoModules[module];
    _mosi = _mosiModules[module];
    _ss = _ssModules[module];
    if (_hasGdo0Modules) _gdo0 = _gdo0Modules[module];
    if (_hasGdo2Modules) _gdo2 = _gdo2Modules[module];
    if (_gdo0 != 255) pinMode(_gdo0, INPUT);
    if (_gdo2 != 255) pinMode(_gdo2, INPUT);
}

bool CC1101Driver::beginTransaction() {
    if (_transactionActive) return true;
    SPIClass *spi = getSPIinstance();
    pinMode(_ss, OUTPUT);
    digitalWrite(_ss, HIGH);
    if (_beginEndLogic) spi->begin(_sck, _miso, _mosi, _ss);
    spi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_ss, LOW);
    if (_miso != 255) {
        uint32_t start = micros();
        while (digitalRead(_miso) && (micros() - start < 1500)) {
            // wait for MISO to go low (CHIP_RDYn)
        }
    }
    _transactionActive = true;
    return true;
}

void CC1101Driver::endTransaction() {
    if (!_transactionActive) return;
    digitalWrite(_ss, HIGH);
    getSPIinstance()->endTransaction();
    if (_beginEndLogic) getSPIinstance()->end();
    _transactionActive = false;
}

bool CC1101Driver::reset() {
    pinMode(_ss, OUTPUT);
    digitalWrite(_ss, HIGH);
    delayMicroseconds(50);
    digitalWrite(_ss, LOW);
    delayMicroseconds(50);
    digitalWrite(_ss, HIGH);
    delayMicroseconds(50);
    if (!beginTransaction()) return false;
    getSPIinstance()->transfer(CC1101_SRES);
    endTransaction();
    delay(1);
    return true;
}

bool CC1101Driver::SpiWriteReg(byte addr, byte value) {
    if (!beginTransaction()) return false;
    getSPIinstance()->transfer(addr);
    getSPIinstance()->transfer(value);
    endTransaction();
    return true;
}

bool CC1101Driver::SpiWriteBurstReg(byte addr, byte *buffer, byte count) {
    if (!buffer || !beginTransaction()) return false;
    getSPIinstance()->transfer(addr | kWriteBurst);
    for (byte i = 0; i < count; i++) getSPIinstance()->transfer(buffer[i]);
    endTransaction();
    return true;
}

bool CC1101Driver::SpiStrobe(byte strobe) {
    if (!beginTransaction()) return false;
    getSPIinstance()->transfer(strobe);
    endTransaction();
    return true;
}

byte CC1101Driver::SpiReadReg(byte addr) {
    if (!beginTransaction()) return 0;
    getSPIinstance()->transfer(addr | kReadSingle);
    byte value = getSPIinstance()->transfer(0);
    endTransaction();
    return value;
}

byte CC1101Driver::SpiReadStatus(byte addr) {
    if (!beginTransaction()) return 0;
    getSPIinstance()->transfer(addr | kReadBurst);
    byte value = getSPIinstance()->transfer(0);
    endTransaction();
    return value;
}

bool CC1101Driver::SpiReadBurstReg(byte addr, byte *buffer, byte count) {
    if (!buffer || !beginTransaction()) return false;
    getSPIinstance()->transfer(addr | kReadBurst);
    for (byte i = 0; i < count; i++) buffer[i] = getSPIinstance()->transfer(0);
    endTransaction();
    return true;
}

bool CC1101Driver::Init() {
    pinMode(_ss, OUTPUT);
    digitalWrite(_ss, HIGH);
    if (_beginEndLogic) getSPIinstance()->begin(_sck, _miso, _mosi, _ss);
    if (!reset()) return false;
    configureDefaults();
    _configured = true;
    return true;
}

void CC1101Driver::configureDefaults() {
    writeConfig(CC1101_FSCTRL1, 0x06);
    writeConfig(CC1101_MDMCFG4, 0x67);
    writeConfig(CC1101_MDMCFG3, 0x32);
    writeConfig(CC1101_MDMCFG2, _mdmcfg2);
    writeConfig(CC1101_MDMCFG1, 0x02);
    writeConfig(CC1101_MDMCFG0, 0xF8);
    writeConfig(CC1101_DEVIATN, 0x47);
    writeConfig(CC1101_MCSM0, 0x18);
    writeConfig(CC1101_FOCCFG, 0x16);
    writeConfig(CC1101_BSCFG, 0x1C);
    writeConfig(CC1101_AGCCTRL2, 0xC7);
    writeConfig(CC1101_AGCCTRL1, 0x00);
    writeConfig(CC1101_AGCCTRL0, 0xB2);
    writeConfig(CC1101_FSCAL3, 0xE9);
    writeConfig(CC1101_FSCAL2, 0x2A);
    writeConfig(CC1101_FSCAL1, 0x00);
    writeConfig(CC1101_FSCAL0, 0x1F);
    writeConfig(CC1101_FREND1, 0x56);
    writeConfig(CC1101_FREND0, 0x11);
    writeConfig(CC1101_TEST2, 0x81);
    writeConfig(CC1101_TEST1, 0x35);
    writeConfig(CC1101_TEST0, 0x09);
    writeConfig(CC1101_PKTCTRL1, _pktctrl1);
    writeConfig(CC1101_PKTCTRL0, _pktctrl0);
    writeConfig(CC1101_ADDR, 0);
    writeConfig(CC1101_PKTLEN, 0);
    setMHZ(_frequency);
}

bool CC1101Driver::writeConfig(byte addr, byte value) { return SpiWriteReg(addr, value); }

void CC1101Driver::setCCMode(bool state) {
    if (state) {
        writeConfig(CC1101_IOCFG2, 0x0B);
        writeConfig(CC1101_IOCFG0, 0x06);
        _pktctrl0 = 0x05;
        writeConfig(CC1101_PKTCTRL0, _pktctrl0);
        writeConfig(CC1101_MDMCFG3, 0xF8);
    } else {
        writeConfig(CC1101_IOCFG2, 0x0D);
        writeConfig(CC1101_IOCFG0, 0x0D);
        _pktctrl0 = 0x32;
        writeConfig(CC1101_PKTCTRL0, _pktctrl0);
        writeConfig(CC1101_MDMCFG3, 0x93);
    }
}

void CC1101Driver::setModulation(byte modulation) {
    _modulation = modulation > 4 ? 4 : modulation;
    const byte modes[] = {0x00, 0x10, 0x30, 0x40, 0x70};
    _mdmcfg2 = (_mdmcfg2 & 0x8F) | modes[_modulation];
    writeConfig(CC1101_MDMCFG2, _mdmcfg2);
    writeConfig(CC1101_FREND0, _modulation == 2 ? 0x11 : 0x10);
}

uint8_t CC1101Driver::paValue(int power) const {
    if (_frequency <= 348.0f) return pa315[constrain(powerIndex(power, 8), 0, 7)];
    if (_frequency <= 464.0f) return pa433[constrain(powerIndex(power, 8), 0, 7)];
    if (_frequency < 900.0f) return pa868[constrain(powerIndex(power, 10), 0, 9)];
    return pa915[constrain(powerIndex(power, 10), 0, 9)];
}

void CC1101Driver::setPA(int power) {
    if (!isFrequencySupported(_frequency)) return;
    _pa = power;
    uint8_t table[8] = {};
    table[0] = _modulation == 2 ? 0 : paValue(power);
    table[1] = _modulation == 2 ? paValue(power) : 0;
    SpiWriteBurstReg(CC1101_PATABLE, table, sizeof(table));
}

void CC1101Driver::setMHZ(float mhz) {
    if (!isFrequencySupported(mhz)) return;
    const uint32_t word = (uint32_t)lroundf(mhz * 1000000.0f * 65536.0f / 26000000.0f);
    if (!SpiWriteReg(CC1101_FREQ2, word >> 16)) return;
    if (!SpiWriteReg(CC1101_FREQ1, word >> 8)) return;
    if (!SpiWriteReg(CC1101_FREQ0, word)) return;
    _frequency = mhz;
    setPA(_pa);
    if (_mode == 1) {
        SetTx();
    } else if (_mode == 2) {
        SetRx();
    }
}

void CC1101Driver::setChannel(byte channel) {
    _channel = channel;
    writeConfig(CC1101_CHANNR, channel);
}

void CC1101Driver::setChsp(float spacing) {
    spacing = constrain(spacing, 25.390625f, 405.456543f);
    byte exponent = 0;
    float mantissa = spacing * 262144.0f / (kXtalMHz * 1000.0f);
    while (mantissa > 511.0f && exponent < 3) {
        mantissa /= 2.0f;
        exponent++;
    }
    byte m = constrain((int)lroundf(mantissa - 256.0f), 0, 255);
    byte current = SpiReadReg(CC1101_MDMCFG1) & 0xFC;
    writeConfig(CC1101_MDMCFG1, current | (exponent & 0x03));
    writeConfig(CC1101_MDMCFG0, m);
}

void CC1101Driver::setRxBW(float bandwidth) {
    bandwidth = constrain(bandwidth, 58.0f, 812.5f);
    float best = 1e9f;
    byte bestM = 0, bestE = 0;
    for (byte e = 0; e < 4; e++) {
        for (byte m = 0; m < 4; m++) {
            float value = (kXtalMHz * 1000.0f) / (8.0f * (4.0f + m) * (1UL << e));
            float diff = fabsf(value - bandwidth);
            if (diff < best) {
                best = diff;
                bestM = m;
                bestE = e;
            }
        }
    }
    byte current = SpiReadReg(CC1101_MDMCFG4) & 0x0F;
    writeConfig(CC1101_MDMCFG4, ((bestE & 0x03) << 6) | ((bestM & 0x03) << 4) | current);
}

void CC1101Driver::setDRate(float rate) {
    rate = constrain(rate, 0.6f, 600.0f);
    byte e = 0;
    float mantissa = rate * 268435456.0f / (kXtalMHz * 1000.0f);
    while (mantissa > 511.0f && e < 15) {
        mantissa /= 2.0f;
        e++;
    }
    byte m = constrain((int)lroundf(mantissa - 256.0f), 0, 255);
    byte cfg = SpiReadReg(CC1101_MDMCFG4) & 0xF0;
    writeConfig(CC1101_MDMCFG4, cfg | (e & 0x0F));
    writeConfig(CC1101_MDMCFG3, m);
}

void CC1101Driver::setDeviation(float deviation) {
    deviation = constrain(deviation, 1.586914f, 380.859375f);
    byte best = 0;
    float error = 1e9f;
    for (byte e = 0; e < 8; e++) {
        for (byte m = 0; m < 8; m++) {
            float value = (kXtalMHz * 1000.0f / 131072.0f) * (8.0f + m) * (1UL << e);
            float diff = fabsf(value - deviation);
            if (diff < error) {
                error = diff;
                best = (e << 4) | m;
            }
        }
    }
    writeConfig(CC1101_DEVIATN, best);
}

void CC1101Driver::SetTx() {
    if (!SpiStrobe(CC1101_SIDLE)) return;
    if (SpiStrobe(CC1101_STX)) _mode = 1;
}
void CC1101Driver::SetRx() {
    if (!SpiStrobe(CC1101_SIDLE)) return;
    if (SpiStrobe(CC1101_SRX)) _mode = 2;
}
void CC1101Driver::SetTx(float mhz) { setSidle(); setMHZ(mhz); SetTx(); }
void CC1101Driver::SetRx(float mhz) { setSidle(); setMHZ(mhz); SetRx(); }

int CC1101Driver::getRssi() {
    if (_mode != 1 && (SpiReadStatus(CC1101_MARCSTATE) & 0x1F) != 0x0D) {
        SetRx();
        delayMicroseconds(150);
    }
    byte value = SpiReadStatus(CC1101_RSSI);
    return value >= 128 ? ((int)value - 256) / 2 - 74 : (int)value / 2 - 74;
}

byte CC1101Driver::getLqi() { return SpiReadStatus(CC1101_LQI); }
void CC1101Driver::setSres() { if (SpiStrobe(CC1101_SRES)) _mode = 0; }
void CC1101Driver::setSidle() { if (SpiStrobe(CC1101_SIDLE)) _mode = 0; }
void CC1101Driver::goSleep() { if (SpiStrobe(CC1101_SIDLE)) SpiStrobe(CC1101_SPWD); _mode = 0; }

bool CC1101Driver::waitForGdo(bool high, uint32_t timeoutMs) {
    if (_gdo0 == 255) return true;
    const uint32_t start = millis();
    while ((digitalRead(_gdo0) != 0) != high) {
        if (millis() - start >= timeoutMs) return false;
        yield();
    }
    return true;
}

void CC1101Driver::SendData(byte *buffer, byte size) {
    if (!buffer || size == 0 || size > kMaxFifoPayload) return;
    if (!SpiWriteReg(CC1101_TXFIFO, size) || !SpiWriteBurstReg(CC1101_TXFIFO, buffer, size)) return;
    setSidle();
    if (!SpiStrobe(CC1101_STX) || !waitForGdo(true, 1000) || !waitForGdo(false, 5000)) {
        setSidle();
        SpiStrobe(CC1101_SFTX);
        return;
    }
    SpiStrobe(CC1101_SFTX);
    _mode = 1;
}

void CC1101Driver::SendData(char *text) { if (text) SendData((byte *)text, (byte)min(strlen(text), (size_t)kMaxFifoPayload)); }
void CC1101Driver::SendData(byte *buffer, byte size, int durationMs) {
    if (!buffer || size == 0 || size > kMaxFifoPayload || durationMs < 0) return;
    if (!SpiWriteReg(CC1101_TXFIFO, size) || !SpiWriteBurstReg(CC1101_TXFIFO, buffer, size)) return;
    setSidle();
    if (SpiStrobe(CC1101_STX)) delay(min(durationMs, 5000));
    setSidle();
    SpiStrobe(CC1101_SFTX);
}
void CC1101Driver::SendData(char *text, int durationMs) { if (text) SendData((byte *)text, (byte)min(strlen(text), (size_t)kMaxFifoPayload), durationMs); }

byte CC1101Driver::CheckReceiveFlag() {
    if (_mode != 2) SetRx();
    if (_gdo0 == 255 || !digitalRead(_gdo0)) return 0;
    return waitForGdo(false, 1000) ? 1 : 0;
}

byte CC1101Driver::ReceiveData(byte *buffer) {
    if (!buffer || !(SpiReadStatus(CC1101_RXBYTES) & kRxFifoMask)) return 0;
    byte size = SpiReadReg(CC1101_RXFIFO);
    if (size > kMaxFifoPayload) { SpiStrobe(CC1101_SFRX); SetRx(); return 0; }
    if (!SpiReadBurstReg(CC1101_RXFIFO, buffer, size)) return 0;
    byte status[2];
    SpiReadBurstReg(CC1101_RXFIFO, status, sizeof(status));
    SpiStrobe(CC1101_SFRX);
    SetRx();
    return size;
}

bool CC1101Driver::CheckCRC() {
    if (SpiReadStatus(CC1101_LQI) & 0x80) return true;
    SpiStrobe(CC1101_SFRX);
    SetRx();
    return false;
}

bool CC1101Driver::getCC1101() { return SpiReadStatus(CC1101_VERSION) != 0; }
byte CC1101Driver::getMode() { return _mode; }
void CC1101Driver::setClb(byte, byte, byte) {}
void CC1101Driver::setSyncWord(byte high, byte low) { writeConfig(CC1101_SYNC1, high); writeConfig(CC1101_SYNC0, low); }
void CC1101Driver::setAddr(byte value) { writeConfig(CC1101_ADDR, value); }
void CC1101Driver::setWhiteData(bool enabled) { _pktctrl0 = (_pktctrl0 & ~0x40) | (enabled ? 0x40 : 0); writeConfig(CC1101_PKTCTRL0, _pktctrl0); }
void CC1101Driver::setPktFormat(byte format) { _pktctrl0 = (_pktctrl0 & ~0x30) | ((format & 3) << 4); writeConfig(CC1101_PKTCTRL0, _pktctrl0); }
void CC1101Driver::setCrc(bool enabled) { _pktctrl0 = (_pktctrl0 & ~0x04) | (enabled ? 0x04 : 0); writeConfig(CC1101_PKTCTRL0, _pktctrl0); }
void CC1101Driver::setLengthConfig(byte config) { _pktctrl0 = (_pktctrl0 & ~3) | (config & 3); writeConfig(CC1101_PKTCTRL0, _pktctrl0); }
void CC1101Driver::setPacketLength(byte length) { writeConfig(CC1101_PKTLEN, length); }
void CC1101Driver::setDcFilterOff(bool enabled) { _mdmcfg2 = (_mdmcfg2 & ~0x80) | (enabled ? 0x80 : 0); writeConfig(CC1101_MDMCFG2, _mdmcfg2); }
void CC1101Driver::setManchester(bool enabled) { _mdmcfg2 = (_mdmcfg2 & ~8) | (enabled ? 8 : 0); writeConfig(CC1101_MDMCFG2, _mdmcfg2); }
void CC1101Driver::setSyncMode(byte mode) { _mdmcfg2 = (_mdmcfg2 & ~7) | (mode & 7); writeConfig(CC1101_MDMCFG2, _mdmcfg2); }
void CC1101Driver::setFEC(bool enabled) { byte v = SpiReadReg(CC1101_MDMCFG1); writeConfig(CC1101_MDMCFG1, (v & ~0x80) | (enabled ? 0x80 : 0)); }
void CC1101Driver::setPRE(byte value) { byte v = SpiReadReg(CC1101_MDMCFG1); writeConfig(CC1101_MDMCFG1, (v & ~0x70) | ((value & 7) << 4)); }
void CC1101Driver::setPQT(byte value) { _pktctrl1 = (_pktctrl1 & ~0xE0) | ((value & 7) << 5); writeConfig(CC1101_PKTCTRL1, _pktctrl1); }
void CC1101Driver::setCRC_AF(bool enabled) { _pktctrl1 = (_pktctrl1 & ~8) | (enabled ? 8 : 0); writeConfig(CC1101_PKTCTRL1, _pktctrl1); }
void CC1101Driver::setAppendStatus(bool enabled) { _pktctrl1 = (_pktctrl1 & ~4) | (enabled ? 4 : 0); writeConfig(CC1101_PKTCTRL1, _pktctrl1); }
void CC1101Driver::setAdrChk(byte value) { _pktctrl1 = (_pktctrl1 & ~3) | (value & 3); writeConfig(CC1101_PKTCTRL1, _pktctrl1); }
bool CC1101Driver::CheckRxFifo(int delayMs) { if (_mode != 2) SetRx(); if (SpiReadStatus(CC1101_RXBYTES) & kRxFifoMask) { if (delayMs > 0) delay(delayMs); return true; } return false; }