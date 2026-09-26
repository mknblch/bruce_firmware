# M5Stack Cardputer ADV + Cap LoRa-1262 Integration & Fix Guide

### Executive Summary

This specification provides autonomous AI agents and engineers with complete, reproducible instructions to resolve signal reception failure, UI unresponsiveness, and pin contention on the **M5Stack Cardputer ADV** equipped with the **M5Stack Cap LoRa-1262** (SKU: U214) running the Bruce firmware.

Applying these modifications eliminates wasted radio frequency energy, prevents electrical hardware conflicts, ensures zero-packet-loss reception, and enables seamless hot-swapping between the Cap LoRa-1262 and the Cardputer ADV Multi-Module Hat (CC1101 + NRF24L01+ + IR) without manual configuration.

---

### 1. Hardware Architecture & Root Cause Analysis

#### A. Hardware Specifications & Components
* **Host Platform:** M5Stack Cardputer ADV (ESP32-S3, `TCA8418` I2C keyboard at `0x34` on `Wire1`: `SDA = GPIO 8`, `SCL = GPIO 9`).
* **Addon Module:** M5Stack Cap LoRa-1262 (Semtech SX1262 transceiver + active 3.0 V TCXO + PI4IOE5V6408 IO expander + FM8625H RF switch + GNSS UART).
* **RF Front-End Switch (`FM8625H`):** Controlled by pin **`P0`** of the **`PI4IOE5V6408`** 8-bit I2C IO expander at address **`0x43`** on `Wire1`.
* **Clock & Regulation:** Active **3.0 V TCXO** connected to SX1262 pin `DIO3`. The SX1262 internal power regulation must be configured for **LDO mode** rather than DC-DC mode.

#### B. The Root Causes of Failure
1. **RF Antenna Isolation (Zero Signals Received):**
   * The `FM8625H` SPDT switch isolates the RP-SMA antenna connector when expander pin `P0` is LOW or high-impedance.
   * Without driving `P0` HIGH, the SX1262 receives only internal thermal noise (-135 dBm) with ~30–50 dB attenuation. The SPI bus communicates normally, creating the false illusion that the radio is functional.
2. **Electrical Pin Contention on Shared 14-Pin Expansion Header:**
   * Cardputer ADV exposes a single 14-pin expansion header.
   * Board defaults in `interface.cpp` previously configured `NRF24_bus.cs = 6` as `OUTPUT HIGH` and `CC1101_bus.io0 = 5`.
   * On Cap LoRa-1262, `GPIO 6` is the SX1262 `BUSY` line (an output from the radio) and `GPIO 5` is `NSS/CS`. Driving `GPIO 6` as an output causes active electrical bus fighting, risking driver burnout and freezing SPI transfers.
3. **TCXO Undervoltage & Crystal Drift:**
   * RadioLib's `SX1262::begin(freq)` defaults to `tcxoVoltage = 1.6f` and `useRegulatorLDO = false`.
   * Supplying 1.6 V to an active 3.0 V TCXO causes oscillator instability, PLL lock failures, and massive frequency offset.
4. **RadioLib 50 ms Fallback Delay on Missing BUSY Pin:**
   * When `LORA_BUSY` is unassigned (`RADIOLIB_NC`), RadioLib injects a blocking `delay(50)` on every SPI transaction requiring a BUSY check.
   * This causes initialization to take >4.5 seconds, channel hops to stall, and scanner navigation to drop keystrokes against FreeRTOS's 75 ms input timeout.
5. **RF Center Frequency Mismatch:**
   * Narrowband LoRa (e.g., BW 31.25 kHz) only tolerates ~±15 kHz carrier offset. Transmitting on 868.000 MHz while listening on 868.100 MHz places the signal completely outside the receiver's digital channel filter.

---

### 2. Header Pinout & Peripheral Collision Matrix

The table below contrasts the Cap LoRa-1262 pinout with the Multi-Module Hat (CC1101 + NRF24 + IR) across the shared 14-pin header:

| Physical Pin | Cap LoRa-1262 Function | Multi-Module Hat Function | Impact if Simultaneously Configured | Required Arbitration |
|---|---|---|---|---|
| `GPIO 40` | `SCK` (SPI Clock) | `SCK` (SPI Clock) | Shared bus | Shared |
| `GPIO 39` | `MISO` (SPI Data Out) | `MISO` (SPI Data Out) | Shared bus | Shared |
| `GPIO 14` | `MOSI` (SPI Data In) | `MOSI` (SPI Data In) | Shared bus | Shared |
| `GPIO 3` | SX1262 `RESET` | NRF24 `CE` (`io0`) | Toggling NRF resets LoRa | Mutual exclusion |
| `GPIO 4` | SX1262 `IRQ` (`DIO1`) | NRF24 `CS` (`CSN`) | Driving CS fights LoRa IRQ | Mutual exclusion |
| `GPIO 5` | SX1262 `NSS` (`CS`) | CC1101 `io0` / IR `RX` | IR/CC1101 pulls LoRa CS | Mutual exclusion |
| `GPIO 6` | SX1262 `BUSY` (Output) | NRF24 `CS` / IR `TX` | **Dangerous contention on pin 6** | Mutual exclusion |
| `GPIO 13` | Cap GNSS `RX` | CC1101 `io0` (`GDO0`) | GNSS serial corrupts CC1101 | Mutual exclusion |
| `GPIO 15` | Cap GNSS `TX` | CC1101 `CS` | UART TX pulls CC1101 CS | Mutual exclusion |
| `GPIO 8` | System I2C `SDA` | System I2C `SDA` | Expander `0x43`, Keypad `0x34` | Shared |
| `GPIO 9` | System I2C `SCL` | System I2C `SCL` | Expander `0x43`, Keypad `0x34` | Shared |

---

### 3. Step-by-Step Code Modifications

To implement the complete fix on any branch, apply the following 6 code changes:

#### Step 1: Add Compile-Time Pin Flags
**File:** `bruce/boards/m5stack-cardputer/m5stack-cardputer.ini`  
Ensure `LORA_IRQ` and `LORA_BUSY` are passed into the build flags:

```ini
	;LoRa
	-DLORA_SCK=40
	-DLORA_MISO=39
	-DLORA_MOSI=14
	-DLORA_CS=5
	-DLORA_RST=3
	-DLORA_DIO0=4
	-DLORA_IRQ=4
	-DLORA_BUSY=6
```

---

#### Step 2: Support `LORA_BUSY` in Pin Config Struct
**File:** `bruce/src/core/configPins.h`  
Update the `LoRa_bus` macro instantiation to populate the `io1` (BUSY) line when `LORA_BUSY` is defined:

```cpp
#ifdef LORA_SCK
#ifdef LORA_BUSY
    SPIPins LoRa_bus = {
        (gpio_num_t)LORA_SCK,
        (gpio_num_t)LORA_MISO,
        (gpio_num_t)LORA_MOSI,
        (gpio_num_t)LORA_CS,
        (gpio_num_t)LORA_RST,
        (gpio_num_t)LORA_DIO0,
        (gpio_num_t)LORA_BUSY
    };
#else
    SPIPins LoRa_bus = {
        (gpio_num_t)LORA_SCK,
        (gpio_num_t)LORA_MISO,
        (gpio_num_t)LORA_MOSI,
        (gpio_num_t)LORA_CS,
        (gpio_num_t)LORA_RST,
        (gpio_num_t)LORA_DIO0
    };
#endif
#else
    SPIPins LoRa_bus;
#endif
```

---

#### Step 3: Implement Frontend Activation & Dynamic Hardware Arbitration
**File:** `bruce/boards/m5stack-cardputer/interface.cpp`  
1. Implement `prepareCardputerLoRaFrontend()` to configure the `PI4IOE5V6408` expander and drive `P0` HIGH.
2. Provide the board-level `prepareBoardLoRaRadio()` hook.
3. In `_post_setup_gpio()`, dynamically arbitrate pins based on whether address `0x43` is detected on `Wire1`.

```cpp
bool prepareCardputerLoRaFrontend() {
    TwoWire *wire = getSysI2CBus();
    if (wire == nullptr) {
        wire = &Wire1;
    }
    lockSysI2CBus();
    uint8_t capExpanderId = 0;
    if (!readI2CRegister8(*wire, 0x43, 0x01, capExpanderId) || capExpanderId == 0) {
        unlockSysI2CBus();
        return true;
    }
    // Configure expander: Output direction, disable high-Z, drive P0 HIGH
    writeI2CRegister8(*wire, 0x43, 0x01, 0x01);
    bool ok = updateI2CRegisterBit(*wire, 0x43, 0x03, 0x01, true) &&
              updateI2CRegisterBit(*wire, 0x43, 0x07, 0x01, false) &&
              updateI2CRegisterBit(*wire, 0x43, 0x05, 0x01, true);
    unlockSysI2CBus();
    Serial.printf("CAP LoRa-1262 frontend RF switch %s\n", ok ? "enabled" : "failed");
    return ok;
}

bool prepareBoardLoRaRadio() {
    return prepareCardputerLoRaFrontend();
}
```

In `_post_setup_gpio()`:
```cpp
    if (capLoRa1262Detected) {
        // Assign LoRa pins
        bruceConfigPins.LoRa_bus.sck = (gpio_num_t)40;
        bruceConfigPins.LoRa_bus.miso = (gpio_num_t)39;
        bruceConfigPins.LoRa_bus.mosi = (gpio_num_t)14;
        bruceConfigPins.LoRa_bus.cs = (gpio_num_t)5;
        bruceConfigPins.LoRa_bus.io0 = (gpio_num_t)3; // RST
        bruceConfigPins.LoRa_bus.io1 = (gpio_num_t)6; // BUSY
        bruceConfigPins.LoRa_bus.io2 = (gpio_num_t)4; // IRQ

        // Suppress conflicting pins in RAM to protect against bus contention
        if (bruceConfigPins.NRF24_bus.cs == 6 || bruceConfigPins.NRF24_bus.cs == 4) {
            bruceConfigPins.NRF24_bus.cs = GPIO_NUM_NC;
        }
        if (bruceConfigPins.NRF24_bus.io0 == 4 || bruceConfigPins.NRF24_bus.io0 == 3) {
            bruceConfigPins.NRF24_bus.io0 = GPIO_NUM_NC;
        }
        if (bruceConfigPins.CC1101_bus.cs == 13 || bruceConfigPins.CC1101_bus.cs == 15 || bruceConfigPins.CC1101_bus.cs == 5) {
            bruceConfigPins.CC1101_bus.cs = GPIO_NUM_NC;
        }
        if (bruceConfigPins.CC1101_bus.io0 == 5 || bruceConfigPins.CC1101_bus.io0 == 13 || bruceConfigPins.CC1101_bus.io0 == 15) {
            bruceConfigPins.CC1101_bus.io0 = GPIO_NUM_NC;
        }
        if (bruceConfigPins.irTx == 5 || bruceConfigPins.irTx == 6 || bruceConfigPins.irTx == 3 || bruceConfigPins.irTx == 4) {
            bruceConfigPins.irTx = 44;
        }
        if (bruceConfigPins.irRx == 5 || bruceConfigPins.irRx == 6 || bruceConfigPins.irRx == 3 || bruceConfigPins.irRx == 4) {
            bruceConfigPins.irRx = -1;
        }
    } else {
        // LoRa cap not detected: retain default CC1101 and NRF24 pin allocations
        bruceConfigPins.CC1101_bus.sck = (gpio_num_t)40;
        bruceConfigPins.CC1101_bus.miso = (gpio_num_t)39;
        bruceConfigPins.CC1101_bus.mosi = (gpio_num_t)14;
        bruceConfigPins.CC1101_bus.cs = (gpio_num_t)13;
        bruceConfigPins.CC1101_bus.io0 = (gpio_num_t)5;

        bruceConfigPins.NRF24_bus.sck = (gpio_num_t)40;
        bruceConfigPins.NRF24_bus.miso = (gpio_num_t)39;
        bruceConfigPins.NRF24_bus.mosi = (gpio_num_t)14;
        bruceConfigPins.NRF24_bus.cs = (gpio_num_t)6;
        bruceConfigPins.NRF24_bus.io0 = (gpio_num_t)4;
    }
```

---

#### Step 4: Correct SX1262 RadioLib Initialization
**File:** `bruce/src/modules/lora/LoRaRadio.cpp`  
Initialize SX1262 with explicit 3.0 V TCXO voltage, LDO mode, DIO2 RF switch control, and Boosted RX gain:

```cpp
    } else {
        gLora1262 = new SX1262(gLoraModule);
        // Explicitly set 3.0V TCXO voltage and useRegulatorLDO = true
        state = gLora1262->begin(
            cfg.freqMHz, cfg.bwKHz, cfg.sf, cfg.cr, cfg.syncWord, cfg.powerDbm, cfg.preambleLen, 3.0f, true
        );
        if (state != RADIOLIB_ERR_NONE) {
            Serial.printf("[LoRa] SX1262 begin with 3.0V TCXO/LDO failed (%d), falling back to default begin\n", state);
            state = gLora1262->begin(cfg.freqMHz);
            if (state == RADIOLIB_ERR_NONE) state = gLora1262->setSpreadingFactor(cfg.sf);
            if (state == RADIOLIB_ERR_NONE) state = gLora1262->setBandwidth(cfg.bwKHz);
            if (state == RADIOLIB_ERR_NONE) state = gLora1262->setCodingRate(cfg.cr);
            if (state == RADIOLIB_ERR_NONE) state = gLora1262->setSyncWord(cfg.syncWord);
            if (state == RADIOLIB_ERR_NONE) state = gLora1262->setPreambleLength(cfg.preambleLen);
            if (state == RADIOLIB_ERR_NONE) state = gLora1262->setOutputPower(cfg.powerDbm);
        }
        if (state == RADIOLIB_ERR_NONE) state = gLora1262->setDio2AsRfSwitch(true);
        if (state == RADIOLIB_ERR_NONE) gLora1262->setRxBoostedGainMode(true);
        if (state == RADIOLIB_ERR_NONE) state = gLora1262->setCRC(true);
        if (state == RADIOLIB_ERR_NONE) state = gLora1262->explicitHeader();
        ...
```

Also verify the following performance optimizations in `LoRaRadio.cpp`:
* **Instant RSSI:** Use `getRSSI(false)` in `getLoRaInstantRSSI()` to sample real-time RF noise floor rather than returning `0.0 dBm`.
* **Instant PLL Retuning:** Use `gLora1262->setFrequency(freqMHz, true)` in `setLoRaFrequency()` to tune frequency without blocking image calibration.
* **Non-Blocking Packet Check:** In `checkLoRaPacketAvailable()`, check the volatile `gLoraPacketReceived` flag directly instead of issuing SPI status queries.

---

#### Step 5: Update Scanner Preset Channels & Responsiveness
**File:** `bruce/src/modules/lora/LoRaScanner.cpp`  
Add dual test frequencies (868.000 MHz and 868.100 MHz) to prevent missed packets during SDR or node testing:

```cpp
        {"Bruce 868 Test (SF9/BW31)", [&]() {
            channels = {
                {"Bruce 868.0", 868.000f, 9, 31.25f, 0, -140.0f, -140.0f, 0},
                {"Bruce 868.1", 868.100f, 9, 31.25f, 0, -140.0f, -140.0f, 0},
            };
            scanCr = 8;
            scanSyncWord = 0x12;
            scanPreambleLen = 8;
        }},
```

Ensure UI navigation synchronizes `KeyStroke.Clear()` when navigation flags are consumed to eliminate phantom keystrokes.

---

#### Step 6: Configuration Persistence (`brucePins.conf`)
**File:** `brucePins.conf` (SD Card or LittleFS root)  
Maintain the persistent configuration schema. Populating both modules allows seamless hot-swapping:

```json
{
  "AC:A7:04:02:BE:DC": {
    "rot": 1,
    "irTx": 44,
    "irRx": -1,
    "CC1101_Pins": {
      "sck": 40,
      "miso": 39,
      "mosi": 14,
      "cs": 15,
      "io0": 13,
      "io2": -1
    },
    "NRF24_Pins": {
      "sck": 40,
      "miso": 39,
      "mosi": 14,
      "cs": 4,
      "io0": 3,
      "io2": -1
    },
    "LoRa_Pins": {
      "sck": 40,
      "miso": 39,
      "mosi": 14,
      "cs": 5,
      "io0": 3,
      "io1": 6,
      "io2": 4
    }
  }
}
```

---

### 4. Flashing & Offset Requirements

When flashing compiled binaries to the ESP32-S3 via `esptool`:
* Standard Bruce application partition offset is **`0x10000`** (64 KB).
* Check `bruce/flash.sh` and ensure default `OFFSET` is configured to `0x10000`:
  ```bash
  OFFSET="${OFFSET:-0x10000}"
  esptool --chip esp32s3 -p "$PORT" -b 921600 write-flash "$OFFSET" "$BIN"
  ```

---

### 5. Verification Protocol & Quality Gates

To confirm the integration is successful without transmitting or causing RF interference:

#### Gate 1: Boot Log Verification
Connect to the serial console at 115200 baud and boot the Cardputer ADV. Confirm:
```text
DEBUG: Cardputer ADV - Initializing TCA8418 keyboard
TCA8418 keyboard initialized successfully
CAP LoRa-1262 frontend RF switch enabled
CAP LoRa-1262 default pins assigned
```

#### Gate 2: Radio Status & Pin Mapping Check
Run the CLI command:
```text
lora status
```
Confirm:
* Module: `SX1262`
* SPI Pins: `SCK=40, MISO=39, MOSI=14, CS=5`
* Control Pins: `RST=3, BUSY=6, IRQ=4`
* No warnings regarding missing BUSY pin or 50 ms fallback delays.

#### Gate 3: Live Benchmark (`lora bench`)
Run the receive-only benchmark:
```text
lora bench
```
Confirm expected metrics:
* **RX initialization:** < 50,000 µs (down from >4,500,000 µs without BUSY pin).
* **Instant RSSI average:** ~75–150 µs (down from 51,000 µs).
* **Frequency Retune:** < 200 µs (instant PLL lock).
* **Noise Floor:** Reads between **-120.0 dBm** and **-138.0 dBm** (confirms RF switch is closed and antenna path is active; an unclosed switch reads -140 to -145 dBm pure thermal floor with no ambient variation).

#### Gate 4: Over-The-Air Reception Test
Open `Radio -> LoRa -> Channel Detector / Scanner` on Cardputer ADV:
* Highlight navigation with `;` and `.` responds immediately on the first keypress.
* Transmit a test frame (e.g., using `libre_sdr_lora_tx.py` at 868.000 MHz, SF9, BW 31.25 kHz, Sync 0x12).
* The packet is immediately detected, hit counter increments, and RSSI displays accurate received signal strength (typically -40 dBm to -90 dBm).
