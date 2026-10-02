#ifndef ELECHOUSE_CC1101_SRC_DRV_H
#define ELECHOUSE_CC1101_SRC_DRV_H

#include <Arduino.h>
#include <SPI.h>

#define CC1101_IOCFG2 0x00
#define CC1101_IOCFG1 0x01
#define CC1101_IOCFG0 0x02
#define CC1101_FIFOTHR 0x03
#define CC1101_SYNC1 0x04
#define CC1101_SYNC0 0x05
#define CC1101_PKTLEN 0x06
#define CC1101_PKTCTRL1 0x07
#define CC1101_PKTCTRL0 0x08
#define CC1101_ADDR 0x09
#define CC1101_CHANNR 0x0A
#define CC1101_FSCTRL1 0x0B
#define CC1101_FSCTRL0 0x0C
#define CC1101_FREQ2 0x0D
#define CC1101_FREQ1 0x0E
#define CC1101_FREQ0 0x0F
#define CC1101_MDMCFG4 0x10
#define CC1101_MDMCFG3 0x11
#define CC1101_MDMCFG2 0x12
#define CC1101_MDMCFG1 0x13
#define CC1101_MDMCFG0 0x14
#define CC1101_DEVIATN 0x15
#define CC1101_MCSM2 0x16
#define CC1101_MCSM1 0x17
#define CC1101_MCSM0 0x18
#define CC1101_FOCCFG 0x19
#define CC1101_BSCFG 0x1A
#define CC1101_AGCCTRL2 0x1B
#define CC1101_AGCCTRL1 0x1C
#define CC1101_AGCCTRL0 0x1D
#define CC1101_FREND1 0x21
#define CC1101_FREND0 0x22
#define CC1101_FSCAL3 0x23
#define CC1101_FSCAL2 0x24
#define CC1101_FSCAL1 0x25
#define CC1101_FSCAL0 0x26
#define CC1101_FSTEST 0x29
#define CC1101_TEST2 0x2C
#define CC1101_TEST1 0x2D
#define CC1101_TEST0 0x2E

#define CC1101_SRES 0x30
#define CC1101_SCAL 0x33
#define CC1101_SRX 0x34
#define CC1101_STX 0x35
#define CC1101_SIDLE 0x36
#define CC1101_SFRX 0x3A
#define CC1101_SFTX 0x3B
#define CC1101_SPWD 0x39

#define CC1101_PARTNUM 0x30
#define CC1101_VERSION 0x31
#define CC1101_LQI 0x33
#define CC1101_RSSI 0x34
#define CC1101_MARCSTATE 0x35
#define CC1101_PKTSTATUS 0x38
#define CC1101_TXBYTES 0x3A
#define CC1101_RXBYTES 0x3B
#define CC1101_PATABLE 0x3E
#define CC1101_TXFIFO 0x3F
#define CC1101_RXFIFO 0x3F

class CC1101Driver {
public:
    bool Init();
    byte SpiReadStatus(byte addr);
    void setBeginEndLogic(bool state);
    bool getBeginEndLogic();
    void setSPIinstance(SPIClass *spi);
    SPIClass *getSPIinstance();
    void setSpiPin(byte sck, byte miso, byte mosi, byte ss);
    void addSpiPin(byte sck, byte miso, byte mosi, byte ss, byte module);
    void setGDO(byte gdo0, byte gdo2);
    void setGDO0(byte gdo0);
    void addGDO(byte gdo0, byte gdo2, byte module);
    void addGDO0(byte gdo0, byte module);
    void setModul(byte module);
    void setCCMode(bool state);
    void setModulation(byte modulation);
    void setPA(int power);
    void setMHZ(float mhz);
    void setChannel(byte channel);
    void setChsp(float spacing);
    void setRxBW(float bandwidth);
    void setDRate(float rate);
    void setDeviation(float deviation);
    void SetTx();
    void SetRx();
    void SetTx(float mhz);
    void SetRx(float mhz);
    int getRssi();
    byte getLqi();
    void setSres();
    void setSidle();
    void goSleep();
    void SendData(byte *buffer, byte size);
    void SendData(char *text);
    void SendData(byte *buffer, byte size, int durationMs);
    void SendData(char *text, int durationMs);
    byte CheckReceiveFlag();
    byte ReceiveData(byte *buffer);
    bool CheckCRC();
    bool SpiStrobe(byte strobe);
    bool SpiWriteReg(byte addr, byte value);
    bool SpiWriteBurstReg(byte addr, byte *buffer, byte count);
    byte SpiReadReg(byte addr);
    bool SpiReadBurstReg(byte addr, byte *buffer, byte count);
    void setClb(byte band, byte start, byte end);
    bool getCC1101();
    byte getMode();
    void setSyncWord(byte high, byte low);
    void setAddr(byte value);
    void setWhiteData(bool enabled);
    void setPktFormat(byte format);
    void setCrc(bool enabled);
    void setLengthConfig(byte config);
    void setPacketLength(byte length);
    void setDcFilterOff(bool enabled);
    void setManchester(bool enabled);
    void setSyncMode(byte mode);
    void setFEC(bool enabled);
    void setPRE(byte value);
    void setPQT(byte value);
    void setCRC_AF(bool enabled);
    void setAppendStatus(bool enabled);
    void setAdrChk(byte value);
    bool CheckRxFifo(int delayMs);

    static bool isFrequencySupported(float mhz);

private:
    bool beginTransaction();
    void endTransaction();
    bool reset();
    bool writeConfig(byte addr, byte value);
    void configureDefaults();
    bool waitForGdo(bool high, uint32_t timeoutMs);
    bool validModule(byte module) const;
    uint8_t paValue(int power) const;

    SPIClass _defaultSpi;
    SPIClass *_spi = nullptr;
    bool _beginEndLogic = false;
    bool _transactionActive = false;
    bool _configured = false;
    byte _sck = 18, _miso = 19, _mosi = 23, _ss = 5;
    byte _gdo0 = 255, _gdo2 = 255;
    byte _sckModules[6] = {}, _misoModules[6] = {}, _mosiModules[6] = {}, _ssModules[6] = {};
    byte _gdo0Modules[6] = {}, _gdo2Modules[6] = {};
    bool _hasModulePins = false;
    bool _hasGdo0Modules = false;
    bool _hasGdo2Modules = false;
    byte _module = 0;
    byte _modulation = 2;
    byte _channel = 0;
    int _pa = 12;
    byte _mode = 0;
    float _frequency = 433.92f;
    byte _pktctrl1 = 0x04;
    byte _pktctrl0 = 0x32;
    byte _mdmcfg2 = 0x30;
};

using ELECHOUSE_CC1101 = CC1101Driver;
extern CC1101Driver ELECHOUSE_cc1101;

#endif