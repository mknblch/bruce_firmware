#include "nrf_spectrum.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/spectrum_plot.h"

#define CHANNELS 80
uint8_t channel[CHANNELS];

// Sweeps the whole 2.4GHz band once and updates the smoothed per-channel
// levels. Drawing lives in nrf_draw() so the WebUI can scan without a screen.
String scanChannels(bool web, int multiplier) {
    String result = "{";

    uint8_t rpdValues[CHANNELS] = {0};
    int step = constrain(25 * multiplier, 0, 100);

    for (int i = 0; i < CHANNELS; i++) {
        if (EscPress || AnyKeyPress) break;
        NRFradio.setChannel(i);
        NRFradio.startListening();
        delayMicroseconds(130);
        int rpd = NRFradio.testRPD() ? 1 : 0;
        NRFradio.stopListening();

        if (rpd) {
            int newLvl = channel[i] + step;
            if (newLvl > 100) newLvl = 100;
            channel[i] = (uint8_t)newLvl;
        } else {
            channel[i] = (uint8_t)((channel[i] * 3) / 4);
        }
        rpdValues[i] = channel[i];
    }

    if (web) {
        for (int i = 0; i < CHANNELS; i++) {
            if (i > 0) result += ",";
            result += String(rpdValues[i]);
        }
        result += "}";
    }
    return result; // "{1,32,45,...}" with 80 values, for the WebUI
}

// Spreads the 80 channel levels across the plot columns, interpolating between
// carriers so the trace reads as a continuous band instead of 80 blocks.
static void nrf_envelope(const uint8_t *lvl, uint8_t *env, int plotW) {
    for (int i = 0; i < plotW; i++) {
        int32_t pos = (int32_t)i * (CHANNELS - 1) * 256 / (plotW - 1);
        int ci = pos >> 8;
        int frac = pos & 0xff;
        if (ci >= CHANNELS - 1) {
            ci = CHANNELS - 2;
            frac = 256;
        }
        int v = lvl[ci] + (lvl[ci + 1] - lvl[ci]) * frac / 256;
        env[i] = (uint8_t)constrain(v, 0, 100);
    }
}

void nrf_spectrum() {
    SpectrumPlot plot;
    if (!plot.begin("NRF Spectrum")) {
        displayError("Out of memory", true);
        return;
    }

    const int plotW = plot.width();
    uint8_t *env = (uint8_t *)malloc(plotW);
    uint8_t *envPeak = (uint8_t *)malloc(plotW);
    uint8_t peak[CHANNELS] = {0};
    if (!env || !envPeak) {
        free(env);
        free(envPeak);
        plot.end();
        displayError("Out of memory", true);
        return;
    }

    // 2.400GHz to 2.479GHz, one tick every 20 channels
    const int tickCount = 5;
    int cols[tickCount];
    String labels[tickCount];
    for (int i = 0; i < tickCount; i++) {
        int ch = i * (CHANNELS - 1) / (tickCount - 1);
        cols[i] = ch * (plotW - 1) / (CHANNELS - 1);
        labels[i] = String(2.400f + ch * 0.001f, 2);
    }
    plot.ruler(cols, labels, tickCount);
    plot.status("starting radio...");

    if (!nrf_start(NRF_MODE_SPI)) { // This function only works on SPI
        Serial.println("Fail Starting radio");
        free(env);
        free(envPeak);
        plot.end();
        displayError("NRF24 not found");
        delay(500);
        return;
    }

    NRFradio.setAutoAck(false);
    NRFradio.disableCRC();       // accept any signal we find
    NRFradio.setAddressWidth(2); // a reverse engineering tactic (not typically recommended)
    const uint8_t noiseAddress[][2] = {
        {0x55, 0x55},
        {0xAA, 0xAA},
        {0xA0, 0xAA},
        {0xAB, 0xAA},
        {0xAC, 0xAA},
        {0xAD, 0xAA}
    };
    for (uint8_t i = 0; i < 6; ++i) { NRFradio.openReadingPipe(i, noiseAddress[i]); }
    NRFradio.setDataRate(RF24_1MBPS);

    uint32_t lastFrame = 0, lastRow = 0;
    int multiplier = 2;
    memset(channel, 0, sizeof(channel));

    while (1) {
        if (check(EscPress)) { break; }

        keyStroke k = _getKeyPress();
        if (k.pressed || !k.word.empty()) {
            for (auto ch : k.word) {
                char lowerKey = tolower(ch);
                if (lowerKey == 'g') {
                    multiplier = (multiplier >= 5) ? 1 : multiplier + 1;
                    memset(channel, 0, sizeof(channel));
                    memset(peak, 0, sizeof(peak));
                }
            }
        }

        scanChannels(false, multiplier);

        int maxCh = 0;
        uint8_t maxLvl = 0;
        for (int i = 0; i < CHANNELS; i++) {
            if (channel[i] > peak[i]) peak[i] = channel[i];
            if (channel[i] > maxLvl) {
                maxLvl = channel[i];
                maxCh = i;
            }
        }

        // A full sweep is far quicker than the panel needs to be repainted, so
        // cap the redraw rate and let the radio keep integrating in between.
        if (millis() - lastFrame >= 40) {
            lastFrame = millis();
            for (int i = 0; i < CHANNELS; i++) {
                if (peak[i] > channel[i]) peak[i]--; // slow decay keeps the hold line readable
            }
            nrf_envelope(channel, env, plotW);
            nrf_envelope(peak, envPeak, plotW);

            // highlight the busiest carrier and its immediate neighbours
            int hlC = maxCh * (plotW - 1) / (CHANNELS - 1);
            int hlSpan = max(2, (2 * (plotW - 1)) / (CHANNELS - 1));
            plot.trace(env, envPeak, hlC - hlSpan, hlC + hlSpan);

            if (millis() - lastRow >= 120) {
                lastRow = millis();
                plot.pushRow(env);
                float peakFreq = 2.400f + maxCh * 0.001f;
                plot.status(
                    "pk: ch" + String(maxCh) + " " + String(peakFreq, 3) + "G " +
                    String(env[hlC]) + "% " + String(multiplier) + "x"
                );
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    NRFradio.stopListening();
    NRFradio.powerDown();
    free(env);
    free(envPeak);
    plot.end();
    returnToMenu = true;
    delay(250);
}
