#if !defined(LITE_VERSION)
#include "LoRaWaterfall.h"
#include "LoRaConfig.h"
#include "LoRaConfigHelpers.h"
#include "LoRaRadio.h"
#include "core/configPins.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/spectrum_plot.h"
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
    if (!env || !disp || !envPeak) {
        free(env);
        free(disp);
        free(envPeak);
        plot.end();
        displayError("Out of memory", true);
        return;
    }
    memset(env, 0, plotW);
    memset(disp, 0, plotW);
    memset(envPeak, 0, plotW);

    QuietModeGuard quietGuard;

    float f_start = gLoraWaterfallStartFreq;
    float f_end = gLoraWaterfallEndFreq;
    if (f_end < f_start) {
        float t = f_start;
        f_start = f_end;
        f_end = t;
    }
    if (fabsf(f_end - f_start) < 0.05f) {
        f_end = f_start + 0.1f;
    }

    LoRaConfigData scanRadioConfig = loraConfig;
    scanRadioConfig.freqMHz = f_start;
    if (!initLoRaRadio(scanRadioConfig, true)) {
        free(env);
        free(disp);
        free(envPeak);
        plot.end();
        displayError("Radio Init Failed", true);
        return;
    }

    auto drawRuler = [&](float s, float e) {
        String labels[3] = {
            String(s, 2),
            String((s + e) * 0.5f, 2),
            String(e, 2)
        };
        int cols[3] = { 0, plotW / 2, plotW - 1 };
        plot.ruler(cols, labels, 3);
    };
    drawRuler(f_start, f_end);

    float range = f_end - f_start;
    float step = 0.1f;
    if (range > 50.0f) step = 5.0f;
    else if (range > 10.0f) step = 1.0f;
    else if (range > 1.0f) step = 0.1f;
    else if (range > 0.1f) step = 0.01f;
    else step = 0.001f;

    AgcMode agcMode = AGC_AUTO;
    float agcFloor = -125.0f;
    float agcPeak = -55.0f;
    float rawRssi[WF_BINS];
    uint8_t bins[WF_BINS];

    uint32_t lastStatus = 0;
    while (!check(EscPress)) {
        bool modeChanged = false;
        if (check(SelPress)) {
            modeChanged = true;
        }

        bool zoomIn = false;
        bool zoomOut = false;
        bool panUp = false;
        bool panDown = false;

        keyStroke k = _getKeyPress();
        if (k.pressed || !k.word.empty()) {
            for (auto ch : k.word) {
                char lowerKey = tolower(ch);
                if (lowerKey == 'g' || lowerKey == 'a' || lowerKey == 's') {
                    modeChanged = true;
                } else if (ch == '+' || ch == '=' || ch == ']') {
                    zoomIn = true;
                } else if (ch == '-' || ch == '_' || ch == '[') {
                    zoomOut = true;
                }
            }
        }

        if (modeChanged) {
            agcMode = (AgcMode)((agcMode + 1) % AGC_MODE_COUNT);
            memset(envPeak, 0, plotW);
            memset(disp, 0, plotW);
        }

        // Sweep WF_BINS frequency points across the band.
        // Suspend the background input handler task during the sweep to eliminate
        // periodic I2C matrix scan EMI and context-switch noise spikes.
        int maxBin = 0;
        float maxRssi = -160.0f;
        float minRssi = 0.0f;
        if (xHandle) vTaskSuspend(xHandle);
        for (int b = 0; b < WF_BINS; b++) {
            float f = f_start + (f_end - f_start) * b / (WF_BINS - 1);
            setLoRaFrequency(f);
            delayMicroseconds(1000);
            float rssi = getLoRaMedianRSSI();

            rawRssi[b] = rssi;
            if (EscPress) break;
        }
        if (xHandle) vTaskResume(xHandle);
        tft.drawPixel(0, 0, 0); // Keep shared SPI display bus happy once per frame before display updates
        delay(8); // yield to allow input handler to process any pending keys

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
        }

        int hlSpan = max(2, plotW / (WF_BINS * 2));
        plot.trace(disp, envPeak, maxCol - hlSpan, maxCol + hlSpan);
        plot.pushRow(env);

        if (millis() - lastStatus >= 300) {
            lastStatus = millis();
            float peakFreq = f_start + (f_end - f_start) * maxBin / (WF_BINS - 1);
            const char *modeNames[] = {"AGC", "1x", "2x", "3x"};
            plot.status(String((int)roundf(maxRssi)) + "dBm @" + String(peakFreq, 2) + "M [" + modeNames[agcMode] + "] SEL:gain");
        }

        // Interactive panning & zooming
        if (check(NextPress)) panUp = true;
        if (check(PrevPress)) panDown = true;

        if (panUp) {
            f_start += step;
            f_end += step;
            gLoraWaterfallStartFreq = f_start;
            gLoraWaterfallEndFreq = f_end;
            drawRuler(f_start, f_end);
            memset(envPeak, 0, plotW);
            memset(disp, 0, plotW);
        } else if (panDown) {
            if (f_start - step >= 100.0f) {
                f_start -= step;
                f_end -= step;
                gLoraWaterfallStartFreq = f_start;
                gLoraWaterfallEndFreq = f_end;
                drawRuler(f_start, f_end);
                memset(envPeak, 0, plotW);
                memset(disp, 0, plotW);
            }
        } else if (zoomIn) {
            float center = (f_start + f_end) * 0.5f;
            float newHalfSpan = (f_end - f_start) * 0.35f;
            if (newHalfSpan >= 0.05f) {
                f_start = max(100.0f, center - newHalfSpan);
                f_end = min(1050.0f, center + newHalfSpan);
                range = f_end - f_start;
                if (range > 50.0f) step = 5.0f;
                else if (range > 10.0f) step = 1.0f;
                else if (range > 1.0f) step = 0.1f;
                else if (range > 0.1f) step = 0.01f;
                else step = 0.001f;
                gLoraWaterfallStartFreq = f_start;
                gLoraWaterfallEndFreq = f_end;
                drawRuler(f_start, f_end);
                memset(envPeak, 0, plotW);
                memset(disp, 0, plotW);
            }
        } else if (zoomOut) {
            float center = (f_start + f_end) * 0.5f;
            float newHalfSpan = (f_end - f_start) * 0.75f;
            if (newHalfSpan <= 100.0f) {
                f_start = max(100.0f, center - newHalfSpan);
                f_end = min(1050.0f, center + newHalfSpan);
                range = f_end - f_start;
                if (range > 50.0f) step = 5.0f;
                else if (range > 10.0f) step = 1.0f;
                else if (range > 1.0f) step = 0.1f;
                else if (range > 0.1f) step = 0.01f;
                else step = 0.001f;
                gLoraWaterfallStartFreq = f_start;
                gLoraWaterfallEndFreq = f_end;
                drawRuler(f_start, f_end);
                memset(envPeak, 0, plotW);
                memset(disp, 0, plotW);
            }
        }
    }

    if (xHandle) vTaskResume(xHandle);
    stopLoRaRadio();
    free(env);
    free(disp);
    free(envPeak);
    plot.end();
}

#endif // !LITE_VERSION
