#if !defined(LITE_VERSION)
#include "LoRaWaterfall.h"
#include "LoRaConfig.h"
#include "LoRaConfigHelpers.h"
#include "LoRaRadio.h"
#include "core/configPins.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/spectrum_plot.h"
#include "core/waterfall_input.h"
#include "core/utils.h"
#include "core/wifi/wifi_common.h"
#include "esp_wifi.h"
#include "globals.h"
#include "modules/ble/ble_common.h"
#if !defined(LITE_VERSION)
#include "modules/ble/BLE_Suite.h"
#endif
#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#if defined(CONFIG_BT_ENABLED)
#include <esp_bt.h>
#endif
#include <math.h>

float gLoraWaterfallStartFreq = 863.0f;
float gLoraWaterfallEndFreq = 870.0f;

static bool gLoraWaterfallGpsSleep = true;
static bool gLoraWaterfallWifiBtSleep = true;

static struct {
    bool inQuietMode = false;
    bool gpsSleeping = false;
    bool wifiBtSleeping = false;
    bool wifiWasConnected = false;
    wifi_mode_t prevWifiMode = WIFI_OFF;
    bool bleWasActive = false;
} s_wfQuietState;

static void gpsEnterSleep() {
    if ((int)bruceConfigPins.gps_bus.tx >= 0 && (int)bruceConfigPins.gps_bus.rx >= 0) {
        pinMode(bruceConfigPins.gps_bus.rx, INPUT);
        HardwareSerial gpsUart(2);
        gpsUart.begin(
            bruceConfigPins.gpsBaudrate, SERIAL_8N1, bruceConfigPins.gps_bus.rx, bruceConfigPins.gps_bus.tx
        );
        delay(20);
        // 1.A optional: Silence NMEA periodic sentence output
        gpsUart.print("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*03\r\n");
        gpsUart.flush();
        delay(20);
        // 1.A: Put CASIC GPS (AT6558/ATGM336H) into Standby/Sleep mode
        gpsUart.print("$PCAS12,1*1F\r\n");
        gpsUart.flush();
        delay(20);
        gpsUart.end();
        pinMode(bruceConfigPins.gps_bus.tx, INPUT);
    }
}

static void gpsWakeup() {
    if ((int)bruceConfigPins.gps_bus.tx >= 0 && (int)bruceConfigPins.gps_bus.rx >= 0) {
        pinMode(bruceConfigPins.gps_bus.rx, INPUT);
        HardwareSerial gpsUart(2);
        gpsUart.begin(
            bruceConfigPins.gpsBaudrate, SERIAL_8N1, bruceConfigPins.gps_bus.rx, bruceConfigPins.gps_bus.tx
        );
        delay(20);
        // 1.B: Wake up CASIC GPS with dummy characters & Normal mode command ($PCAS12,0)
        gpsUart.print("\r\n\r\n$PCAS12,0*1E\r\n");
        gpsUart.flush();
        delay(20);
        // Restore standard NMEA sentences (GGA, GLL, GSA, GSV, RMC, VTG)
        gpsUart.print("$PCAS03,1,1,1,1,1,1,0,0,0,0,,,0,0*03\r\n");
        gpsUart.flush();
        delay(20);
        gpsUart.end();
    }
}

static void enterQuietMode() {
    if (s_wfQuietState.inQuietMode) return;
    s_wfQuietState.inQuietMode = true;

    if (gLoraWaterfallGpsSleep) {
        gpsEnterSleep();
        s_wfQuietState.gpsSleeping = true;
    } else {
        s_wfQuietState.gpsSleeping = false;
    }

    if (gLoraWaterfallWifiBtSleep) {
        s_wfQuietState.wifiBtSleeping = true;
        s_wfQuietState.wifiWasConnected = WiFi.isConnected() || wifiConnected;
        s_wfQuietState.prevWifiMode = WiFi.getMode();
#if !defined(LITE_VERSION)
        s_wfQuietState.bleWasActive = BLEStateManager::isBLEActive() || BLEConnected;
#else
        s_wfQuietState.bleWasActive = BLEConnected;
#endif

        if (s_wfQuietState.prevWifiMode != WIFI_MODE_NULL && s_wfQuietState.prevWifiMode != WIFI_OFF) {
            wifiDisconnect();
        } else {
            WiFi.mode(WIFI_OFF);
        }
        stopBLEStack();
#if defined(CONFIG_BT_ENABLED)
        btStop();
#endif
    } else {
        s_wfQuietState.wifiBtSleeping = false;
    }
}

static void exitQuietMode() {
    if (!s_wfQuietState.inQuietMode) return;

    if (s_wfQuietState.gpsSleeping) {
        gpsWakeup();
        s_wfQuietState.gpsSleeping = false;
    }

    if (s_wfQuietState.wifiBtSleeping) {
#if defined(CONFIG_BT_ENABLED)
        if (s_wfQuietState.bleWasActive) {
            btStart();
        }
#endif
        if (s_wfQuietState.prevWifiMode != WIFI_MODE_NULL && s_wfQuietState.prevWifiMode != WIFI_OFF) {
            WiFi.mode(s_wfQuietState.prevWifiMode);
            if (s_wfQuietState.wifiWasConnected) {
                xTaskCreate(wifiConnectTask, "wifiConnectTask", 4096, NULL, 1, NULL);
            }
        }
        s_wfQuietState.wifiBtSleeping = false;
    }

    s_wfQuietState.inQuietMode = false;
}

struct QuietModeGuard {
    QuietModeGuard() { enterQuietMode(); }
    ~QuietModeGuard() { exitQuietMode(); }
};

#define WF_BINS 64

static inline float getLoRaMedianRSSI() {
    float r1 = getLoRaInstantRSSI();
    delayMicroseconds(60);
    float r2 = getLoRaInstantRSSI();
    delayMicroseconds(60);
    float r3 = getLoRaInstantRSSI();
    if ((r1 <= r2 && r2 <= r3) || (r3 <= r2 && r2 <= r1)) return r2;
    if ((r2 <= r1 && r1 <= r3) || (r3 <= r1 && r1 <= r2)) return r1;
    return r3;
}

enum AgcMode {
    AGC_AUTO = 0,
    AGC_GAIN_1X, // Normal -110 .. -40 dBm
    AGC_GAIN_2X, // Boost -120 .. -60 dBm
    AGC_GAIN_3X, // Max -130 .. -80 dBm
    AGC_MODE_COUNT
};

static void selectPresetSpanMenu() {
    loadLoRaConfig();
    std::vector<Option> spanOptions;

    spanOptions.push_back({"EU868 (863 - 870 MHz)", []() {
        gLoraWaterfallStartFreq = 863.0f;
        gLoraWaterfallEndFreq = 870.0f;
        displaySuccess("Set 863 - 870 MHz");
    }});

    spanOptions.push_back({"US915 (902 - 928 MHz)", []() {
        gLoraWaterfallStartFreq = 902.0f;
        gLoraWaterfallEndFreq = 928.0f;
        displaySuccess("Set 902 - 928 MHz");
    }});

    spanOptions.push_back({"ISM433 (433 - 435 MHz)", []() {
        gLoraWaterfallStartFreq = 433.0f;
        gLoraWaterfallEndFreq = 435.0f;
        displaySuccess("Set 433 - 435 MHz");
    }});

    spanOptions.push_back({"ISM315 (310 - 320 MHz)", []() {
        gLoraWaterfallStartFreq = 310.0f;
        gLoraWaterfallEndFreq = 320.0f;
        displaySuccess("Set 310 - 320 MHz");
    }});

    spanOptions.push_back({"Config +/- 1 MHz", []() {
        gLoraWaterfallStartFreq = max(100.0f, loraConfig.freqMHz - 1.0f);
        gLoraWaterfallEndFreq = min(1050.0f, loraConfig.freqMHz + 1.0f);
        displaySuccess("Span +/- 1 MHz");
    }});

    spanOptions.push_back({"Config +/- 5 MHz", []() {
        gLoraWaterfallStartFreq = max(100.0f, loraConfig.freqMHz - 5.0f);
        gLoraWaterfallEndFreq = min(1050.0f, loraConfig.freqMHz + 5.0f);
        displaySuccess("Span +/- 5 MHz");
    }});

    loopOptions(spanOptions, MENU_TYPE_SUBMENU, "Band Presets");
}

static void promptCustomFrequency(const String &prompt, float &targetFreq) {
    tft.fillScreen(bruceConfig.bgColor);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.3f", targetFreq);
    String input = num_keyboard(buf, 12, prompt.c_str());
    if (input == "" || input == "\x1B") return;

    float f = input.toFloat();
    if (!LoRaConfigHelpers::isValidFrequencyMHz(f)) {
        displayError("Invalid Freq (100-1050 MHz)");
        return;
    }
    targetFreq = f;
    displaySuccess("Freq: " + String(f, 3) + " MHz");
}

void runLoRaWaterfallMenu() {
    loadLoRaConfig();
    int option = 0;
    int idx = 0;

    // Default to a reasonable band matching current lora frequency if uninitialized
    if (gLoraWaterfallStartFreq == 863.0f && gLoraWaterfallEndFreq == 870.0f && loraConfig.freqMHz < 500.0f) {
        gLoraWaterfallStartFreq = 433.0f;
        gLoraWaterfallEndFreq = 435.0f;
    }

select_menu:
    option = 0;
    std::vector<Option> menuOpts = {
        {"Start Waterfall", [&]() { option = 1; }},
        {"Presets / Bands", [&]() { option = 2; }},
        {"Start Freq: " + String(gLoraWaterfallStartFreq, 2) + "M", [&]() { option = 3; }},
        {"End Freq: " + String(gLoraWaterfallEndFreq, 2) + "M", [&]() { option = 4; }},
        {String("GPS Sleep: ") + (gLoraWaterfallGpsSleep ? "[ON]" : "[OFF]"), [&]() {
            gLoraWaterfallGpsSleep = !gLoraWaterfallGpsSleep;
        }},
        {String("WiFi/BT Sleep: ") + (gLoraWaterfallWifiBtSleep ? "[ON]" : "[OFF]"), [&]() {
            gLoraWaterfallWifiBtSleep = !gLoraWaterfallWifiBtSleep;
        }},
        {"Main Menu", [&]() { option = 5; }},
    };

    idx = loopOptions(menuOpts, MENU_TYPE_SUBMENU, "LoRa Waterfall", idx);
    tft.fillScreen(bruceConfig.bgColor);

    if (idx < 0 || option == 5) {
        return;
    } else if (option == 1) {
        runLoRaWaterfall();
        goto select_menu;
    } else if (option == 2) {
        selectPresetSpanMenu();
        goto select_menu;
    } else if (option == 3) {
        promptCustomFrequency("Start Freq (MHz):", gLoraWaterfallStartFreq);
        goto select_menu;
    } else if (option == 4) {
        promptCustomFrequency("End Freq (MHz):", gLoraWaterfallEndFreq);
        goto select_menu;
    } else {
        goto select_menu;
    }
}

void runLoRaWaterfallWithSpan(float startFreqMHz, float endFreqMHz) {
    gLoraWaterfallStartFreq = startFreqMHz;
    gLoraWaterfallEndFreq = endFreqMHz;
    runLoRaWaterfall();
}

// Controls cycled with Next/Prev and locked with Select, as in the RF waterfall.
enum LoraWaterfallControl {
    LORA_WF_CONTROL_FREQ = 0,
    LORA_WF_CONTROL_AGC,
    LORA_WF_CONTROL_SPAN,
    LORA_WF_CONTROL_BW,
    LORA_WF_CONTROL_COUNT
};

#define LORA_WF_MIN_SPAN 0.1f
#define LORA_WF_MAX_SPAN 100.0f
// A waterfall row is committed at this pace rather than once per sweep, so the
// visible history keeps its time span however fast the sweep runs. Sweeps in
// between are max-held into the row so short bursts are never dropped.
#define LORA_WF_ROW_MS 200

// Receiver bandwidths both SX1276 and SX1262 accept in LoRa mode.
static const float loraWaterfallBandwidths[] = {7.8f, 10.4f, 15.6f, 20.8f, 31.25f, 41.7f, 62.5f, 125.0f, 250.0f, 500.0f};
static const int loraWaterfallBandwidthCount = sizeof(loraWaterfallBandwidths) / sizeof(loraWaterfallBandwidths[0]);

static int loraWaterfallBandwidthFor(float bwKHz) {
    int best = 0;
    for (int i = 1; i < loraWaterfallBandwidthCount; i++) {
        if (fabsf(loraWaterfallBandwidths[i] - bwKHz) < fabsf(loraWaterfallBandwidths[best] - bwKHz)) best = i;
    }
    return best;
}

// Tunable range of the radio; RadioLib rejects anything outside it, and a
// failed retune stops the radio altogether.
static void loraWaterfallLimits(LoRaRadioType type, float &lo, float &hi) {
    if (type == LoRaRadioType::SX1262) {
        lo = 150.0f;
        hi = 960.0f;
    } else {
        lo = 137.0f;
        hi = 1020.0f;
    }
}

// Keep the scan inside the radio's range while preserving the span.
static void clampLoraWaterfallRange(float &start, float &end, float lo, float hi) {
    if (end < start) {
        float t = start;
        start = end;
        end = t;
    }
    float span = constrain(end - start, LORA_WF_MIN_SPAN, min(LORA_WF_MAX_SPAN, hi - lo));
    if (start < lo) start = lo;
    if (start + span > hi) start = hi - span;
    end = start + span;
}

static float loraWaterfallPanStep(float range) {
    if (range > 50.0f) return 5.0f;
    if (range > 10.0f) return 1.0f;
    if (range > 1.0f) return 0.1f;
    if (range > 0.1f) return 0.01f;
    return 0.001f;
}

void runLoRaWaterfall() {
    loadLoRaConfig();

    if (!isLoraHardwareConfigured()) {
        displayError("LoRa pins not configured!\nCheck Pin Setup", true);
        return;
    }

    if (!isLoraModulePresent(true)) {
        displayError("LoRa module not found!", true);
        return;
    }

    SpectrumPlot plot;
    if (!plot.begin("", /*sdrWaterfall=*/true, /*waterfallPriority=*/true)) {
        displayError("Out of memory", true);
        return;
    }

    const int plotW = plot.width();
    uint8_t *env = (uint8_t *)malloc(plotW);
    uint8_t *disp = (uint8_t *)malloc(plotW);
    uint8_t *envPeak = (uint8_t *)malloc(plotW);
    uint8_t *rowAcc = (uint8_t *)malloc(plotW); // max of sweeps since the last waterfall row
    if (!env || !disp || !envPeak || !rowAcc) {
        free(env);
        free(disp);
        free(envPeak);
        free(rowAcc);
        plot.end();
        displayError("Out of memory", true);
        return;
    }
    memset(env, 0, plotW);
    memset(disp, 0, plotW);
    memset(envPeak, 0, plotW);
    memset(rowAcc, 0, plotW);

    QuietModeGuard quietGuard;

    float freqLo, freqHi;
    loraWaterfallLimits(loraConfig.radioType, freqLo, freqHi);
    float f_start = gLoraWaterfallStartFreq;
    float f_end = gLoraWaterfallEndFreq;
    clampLoraWaterfallRange(f_start, f_end, freqLo, freqHi);

    LoRaConfigData scanRadioConfig = loraConfig;
    scanRadioConfig.freqMHz = f_start;
    if (!initLoRaRadio(scanRadioConfig, true)) {
        free(env);
        free(disp);
        free(envPeak);
        free(rowAcc);
        plot.end();
        displayError("Radio Init Failed", true);
        return;
    }
    int bandwidthIndex = loraWaterfallBandwidthFor(scanRadioConfig.bwKHz);

    auto drawRuler = [&]() {
        String labels[3] = {String(f_start, 2), String((f_start + f_end) * 0.5f, 2), String(f_end, 2)};
        int cols[3] = {0, plotW / 2, plotW - 1};
        plot.ruler(cols, labels, 3);
    };
    drawRuler();
    plot.status("scanning...");

    float step = loraWaterfallPanStep(f_end - f_start);

    AgcMode agcMode = AGC_AUTO;
    float agcFloor = -125.0f;
    float agcPeak = -55.0f;
    float rawRssi[WF_BINS];
    uint8_t bins[WF_BINS];

    LoraWaterfallControl selectedControl = LORA_WF_CONTROL_FREQ;
    bool controlLocked = false;
    bool radioFailed = false;
    uint32_t lastStatus = 0;
    uint32_t lastRow = millis();
    WaterfallInput pending;
    while (!returnToMenu) {
        // Apply everything collected since the previous frame.
        pollWaterfallInput(pending);
        WaterfallInput in = pending;
        pending = WaterfallInput();

        // Select locks the footer control; Esc unlocks it, then exits when
        // the footer is already unlocked.
        if (in.esc) {
            if (controlLocked) {
                controlLocked = false;
                lastStatus = 0;
            } else {
                break;
            }
        }
        if (in.select & 1) {
            controlLocked = !controlLocked;
            lastStatus = 0;
        }

        bool agcChanged = false;
        bool bandwidthChanged = false;
        int panSteps = in.up - in.down; // dedicated Up/Down buttons always pan
        int zoomSteps = in.zoomIn - in.zoomOut;

        // AGC / gain mode from the Cardputer's g/a/s shortcuts.
        if (in.gain > 0) {
            agcMode = (AgcMode)wrapIndex(agcMode + in.gain, AGC_MODE_COUNT);
            agcChanged = true;
        }

        int nav = in.next - in.prev;
        if (nav != 0) {
            if (!controlLocked) {
                selectedControl = (LoraWaterfallControl)wrapIndex(selectedControl + nav, LORA_WF_CONTROL_COUNT);
                lastStatus = 0;
            } else {
                switch (selectedControl) {
                    case LORA_WF_CONTROL_FREQ: panSteps += nav; break;
                    case LORA_WF_CONTROL_AGC:
                        agcMode = (AgcMode)wrapIndex(agcMode + nav, AGC_MODE_COUNT);
                        agcChanged = true;
                        break;
                    case LORA_WF_CONTROL_SPAN: zoomSteps += nav; break;
                    default: {
                        // Stop at the narrowest / widest filter instead of wrapping.
                        int newIndex = constrain(bandwidthIndex + nav, 0, loraWaterfallBandwidthCount - 1);
                        bandwidthChanged = newIndex != bandwidthIndex;
                        bandwidthIndex = newIndex;
                        break;
                    }
                }
            }
        }

        if (agcChanged) {
            memset(envPeak, 0, plotW);
            memset(disp, 0, plotW);
            lastStatus = 0;
        }

        // Pan or zoom the whole window and refresh the ruler + peak history.
        bool rangeChanged = false;
        if (panSteps != 0) {
            float shift = step * panSteps;
            float span = f_end - f_start;
            f_start += shift;
            f_end = f_start + span;
            clampLoraWaterfallRange(f_start, f_end, freqLo, freqHi);
            rangeChanged = true;
        }
        if (zoomSteps != 0) {
            // Each step halves (in) or doubles (out) the span around its centre.
            float center = (f_start + f_end) * 0.5f;
            float span = f_end - f_start;
            int moves = zoomSteps > 0 ? zoomSteps : -zoomSteps;
            while (moves-- > 0) span = zoomSteps > 0 ? span * 0.5f : span * 2.0f;
            span = constrain(span, LORA_WF_MIN_SPAN, LORA_WF_MAX_SPAN);
            f_start = center - span * 0.5f;
            f_end = center + span * 0.5f;
            clampLoraWaterfallRange(f_start, f_end, freqLo, freqHi);
            rangeChanged = true;
        }
        if (rangeChanged) {
            gLoraWaterfallStartFreq = f_start;
            gLoraWaterfallEndFreq = f_end;
            step = loraWaterfallPanStep(f_end - f_start);
            memset(envPeak, 0, plotW);
            memset(disp, 0, plotW);
            memset(rowAcc, 0, plotW); // half a row from the old range would be misleading
            lastRow = millis();
            drawRuler();
            lastStatus = 0;
        }
        if (bandwidthChanged) {
            if (!setLoRaBandwidth(loraWaterfallBandwidths[bandwidthIndex]) || !startLoRaReceive()) {
                radioFailed = true;
                break;
            }
            memset(envPeak, 0, plotW);
            lastStatus = 0;
        }

        // Sweep WF_BINS frequency points across the band.
        // Suspend the background input handler task during the sweep to eliminate
        // periodic I2C matrix scan EMI and context-switch noise spikes. Input is
        // collected right before and after, and between drawing steps below.
        int maxBin = 0;
        float maxRssi = -160.0f;
        float minRssi = 0.0f;
        if (xHandle) vTaskSuspend(xHandle);
        for (int b = 0; b < WF_BINS; b++) {
            float f = f_start + (f_end - f_start) * b / (WF_BINS - 1);
            if (!setLoRaFrequency(f)) {
                radioFailed = true;
                break;
            }
            delayMicroseconds(500);
            rawRssi[b] = getLoRaMedianRSSI();
        }
        if (xHandle) vTaskResume(xHandle);
        if (radioFailed) break;
        yield();
        pollWaterfallInput(pending);
        tft.drawPixel(0, 0, 0); // Keep shared SPI display bus happy once per frame before display updates

        // Spatial 3-point median filter across frequency bins to completely eliminate
        // isolated 1-bin impulse noise spikes (EMI/clock glitch beating)
        float cleanRssi[WF_BINS];
        for (int b = 0; b < WF_BINS; b++) {
            float prev = (b > 0) ? rawRssi[b - 1] : rawRssi[b];
            float curr = rawRssi[b];
            float next = (b < WF_BINS - 1) ? rawRssi[b + 1] : rawRssi[b];
            float med = curr;
            if ((prev <= curr && curr <= next) || (next <= curr && curr <= prev)) med = curr;
            else if ((curr <= prev && prev <= next) || (next <= prev && prev <= curr)) med = prev;
            else med = next;
            cleanRssi[b] = med;

            if (med > maxRssi) {
                maxRssi = med;
                maxBin = b;
            }
            if (med < minRssi) {
                minRssi = med;
            }
        }

        if (agcMode == AGC_AUTO) {
            // Adaptive Noise Floor tracker
            if (minRssi < agcFloor) {
                agcFloor = agcFloor * 0.7f + minRssi * 0.3f;
            } else {
                agcFloor = agcFloor * 0.96f + minRssi * 0.04f;
            }
            agcFloor = constrain(agcFloor, -135.0f, -60.0f);

            // Adaptive Peak Tracker
            if (maxRssi > agcPeak) {
                agcPeak = agcPeak * 0.6f + maxRssi * 0.4f;
            } else {
                agcPeak = agcPeak * 0.97f + maxRssi * 0.03f;
            }
            agcPeak = constrain(agcPeak, -100.0f, -10.0f);

            // Maintain at least 30 dB dynamic range headroom to prevent thermal noise amplification
            if (agcPeak - agcFloor < 30.0f) {
                agcPeak = agcFloor + 30.0f;
            }
        }

        float floorDbm, peakDbm;
        switch (agcMode) {
            case AGC_GAIN_1X:
                floorDbm = -110.0f;
                peakDbm = -40.0f;
                break;
            case AGC_GAIN_2X:
                floorDbm = -120.0f;
                peakDbm = -60.0f;
                break;
            case AGC_GAIN_3X:
                floorDbm = -130.0f;
                peakDbm = -80.0f;
                break;
            case AGC_AUTO:
            default:
                floorDbm = agcFloor;
                peakDbm = agcPeak;
                break;
        }

        for (int b = 0; b < WF_BINS; b++) {
            float norm = (cleanRssi[b] - floorDbm) / (peakDbm - floorDbm);
            int v = (int)roundf(norm * 100.0f);
            v = constrain(v, 0, 100);
            bins[b] = (uint8_t)v;
        }

        // Interpolate bins across display columns and update peak-hold
        int maxCol = maxBin * (plotW - 1) / (WF_BINS - 1);
        for (int x = 0; x < plotW; x++) {
            float bf = (float)x * (WF_BINS - 1) / (float)(plotW - 1);
            int b0 = (int)bf;
            int b1 = (b0 + 1 < WF_BINS) ? b0 + 1 : b0;
            float frac = bf - (float)b0;
            int v = (int)roundf((1.0f - frac) * bins[b0] + frac * bins[b1]);
            env[x] = (uint8_t)constrain(v, 0, 100);

            if (env[x] >= envPeak[x]) {
                envPeak[x] = env[x];
            } else if (envPeak[x] > 0) {
                envPeak[x]--;
            }

            if (env[x] > disp[x]) {
                disp[x] = (uint8_t)min(100, (int)disp[x] + 18);
            } else if (disp[x] > env[x]) {
                int drop = (disp[x] - env[x] > 8) ? 8 : (disp[x] - env[x]);
                disp[x] -= drop;
            }

            if (env[x] > rowAcc[x]) rowAcc[x] = env[x];
        }

        int hlSpan = max(2, plotW / (WF_BINS * 2));
        plot.trace(disp, envPeak, maxCol - hlSpan, maxCol + hlSpan);
        pollWaterfallInput(pending);
        if (millis() - lastRow >= LORA_WF_ROW_MS) {
            lastRow = millis();
            plot.pushRow(rowAcc);
            memset(rowAcc, 0, plotW);
        }
        pollWaterfallInput(pending);

        if (millis() - lastStatus >= 300) {
            lastStatus = millis();
            const char *modeNames[] = {"AUTO", "1x", "2x", "3x"};
            float bw = loraWaterfallBandwidths[bandwidthIndex];
            String items[LORA_WF_CONTROL_COUNT] = {
                String((f_start + f_end) * 0.5f, 2) + "M",
                "AGC " + String(modeNames[agcMode]),
                "SPAN " + String(f_end - f_start, 2),
                "BW " + String(bw, bw >= 100.0f ? 0 : 1) + "k",
            };
            plot.status(waterfallControlBar(items, LORA_WF_CONTROL_COUNT, selectedControl, controlLocked));
        }

        delay(2); // give lower-priority tasks a slice once per frame
    }

    if (xHandle) vTaskResume(xHandle);
    gLoraWaterfallStartFreq = f_start;
    gLoraWaterfallEndFreq = f_end;
    stopLoRaRadio();
    free(env);
    free(disp);
    free(envPeak);
    free(rowAcc);
    plot.end();
    if (radioFailed) displayError("LoRa radio error", true);
}

#endif // !LITE_VERSION
