#include "rf_waterfall.h"
#include "core/mykeyboard.h"
#include "core/spectrum_plot.h"
float m_rf_waterfall_start_freq = 433.0;
float m_rf_waterfall_end_freq = 435.0;

void rf_waterfall() {
    if (bruceConfigPins.rfModule != CC1101_SPI_MODULE) {
        displayError("Waterfall needs a CC1101!", true);
        return;
    }
    if (!initRfModule("rx", m_rf_waterfall_start_freq)) {
        displayError("CC1101 not found!", true);
        return;
    }

    ELECHOUSE_cc1101.setRxBW(200);
    int option, idx = 0;
select:

    option = 0;
    options = {
        {"Start Waterfall", [&]() { option = 3; }},
        {"Start Freq.",     [&]() { option = 1; }},
        {"End Freq.",       [&]() { option = 2; }},
        {"Main Menu",       [&]() { option = 4; }},
    };
    idx = loopOptions(options, idx);

    tft.fillScreen(0x0);

    if (option == 4) {
        return;
    } else if (option == 3) {
        rf_waterfall_run();
        goto select;
    } else if (option == 1) {
        rf_waterfall_boundary_freq(m_rf_waterfall_start_freq);
        goto select;
    } else if (option == 2) {
        rf_waterfall_boundary_freq(m_rf_waterfall_end_freq);
        goto select;
    }
}
void rf_waterfall_boundary_freq(float &boundary) {
    options = {};
    int ind = 0;
    int arraySize = sizeof(subghz_frequency_list) / sizeof(subghz_frequency_list[0]);
    for (int i = 0; i < arraySize; i++) {
        if (subghz_frequency_list[i] - boundary < 0.1) ind = i;
        String tmp = String(subghz_frequency_list[i], 2) + "Mhz";
        options.push_back({tmp.c_str(), [&boundary, i]() { boundary = subghz_frequency_list[i]; }});
    }
    loopOptions(options, ind);
    options.clear();
}


// ── SDR-style waterfall ─────────────────────────────────────────
// Rebuilt on the shared SpectrumPlot so it matches NRF Spectrum: a live filled
// trace with peak-hold across the top, and a theme-coloured waterfall below fed
// by the very same envelope, so the falls track the waveform above them.
//
// The band is swept in a modest number of bins (not one per pixel): retuning a
// CC1101 needs a few ms for the PLL/RSSI to settle before getRssi() is valid
// (see rf_CC1101_rssi), so sampling every pixel with a sub-ms wait would just
// read the noise floor. We sample WF_BINS points with a real settle and then
// interpolate the envelope across the plot columns for a continuous trace.
#define WF_BINS 64

static inline int getMedianRssi() {
    int r1 = ELECHOUSE_cc1101.getRssi();
    delayMicroseconds(60);
    int r2 = ELECHOUSE_cc1101.getRssi();
    delayMicroseconds(60);
    int r3 = ELECHOUSE_cc1101.getRssi();
    if ((r1 <= r2 && r2 <= r3) || (r3 <= r2 && r2 <= r1)) return r2;
    if ((r2 <= r1 && r1 <= r3) || (r3 <= r1 && r1 <= r2)) return r1;
    return r3;
}

enum AgcMode {
    AGC_AUTO = 0,
    AGC_GAIN_1X, // Normal -95 .. -35 dBm
    AGC_GAIN_2X, // Boost -100 .. -50 dBm
    AGC_GAIN_3X, // Max -105 .. -65 dBm
    AGC_MODE_COUNT
};

void rf_waterfall_run() {
    SpectrumPlot plot;
    if (!plot.begin("", /*sdrWaterfall=*/true, /*waterfallPriority=*/true)) { // SDR colourmap, no title bar to maximize waterfall room
        displayError("Out of memory", true);
        return;
    }

    const int plotW = plot.width();
    uint8_t *env = (uint8_t *)malloc(plotW);     // freshly measured envelope
    uint8_t *disp = (uint8_t *)malloc(plotW);    // eased envelope drawn on top
    uint8_t *envPeak = (uint8_t *)malloc(plotW); // peak-hold line
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

    float f_start = m_rf_waterfall_start_freq;
    float f_end = m_rf_waterfall_end_freq;
    if (f_end < f_start) {
        float t = f_start;
        f_start = f_end;
        f_end = t;
    }

    initRfModule("rx", f_start);
    ELECHOUSE_cc1101.setRxBW(200);

    // Five evenly spaced frequency ticks under the plot; redrawn when panning.
    const int tickCount = 5;
    int cols[tickCount];
    String labels[tickCount];
    auto drawRuler = [&]() {
        for (int i = 0; i < tickCount; i++) {
            cols[i] = i * (plotW - 1) / (tickCount - 1);
            float f = f_start + (f_end - f_start) * i / (tickCount - 1);
            labels[i] = String(f, 2);
        }
        plot.ruler(cols, labels, tickCount);
    };
    drawRuler();
    plot.status("scanning...");

    // Pan step scales with the span, matching the old boundary stepping.
    float range = f_end - f_start;
    float step;
    if (range > 100) step = 10;
    else if (range > 10) step = 1;
    else if (range > 1) step = 0.1f;
    else if (range > 0.1f) step = 0.01f;
    else step = 0.001f;

    AgcMode agcMode = AGC_AUTO;
    float agcFloor = -105.0f;
    float agcPeak = -45.0f;
    int rawRssi[WF_BINS];
    uint8_t bins[WF_BINS];

    uint32_t lastStatus = 0;
    while (!check(EscPress)) {
        // Toggle AGC / gain mode on Select press or 'g' / 'a' / 's' key
        bool modeChanged = false;
        if (check(SelPress)) {
            modeChanged = true;
        }

        keyStroke k = _getKeyPress();
        if (k.pressed || !k.word.empty()) {
            for (auto ch : k.word) {
                char lowerKey = tolower(ch);
                if (lowerKey == 'g' || lowerKey == 'a' || lowerKey == 's') {
                    modeChanged = true;
                }
            }
        }

        if (modeChanged) {
            agcMode = (AgcMode)((agcMode + 1) % AGC_MODE_COUNT);
            memset(envPeak, 0, plotW);
            memset(disp, 0, plotW);
        }

        // Sweep the band once — WF_BINS RSSI samples with a real settle so the
        // reading reflects the tuned frequency instead of the noise floor.
        // Suspend the background input handler task during the sweep to eliminate
        // periodic I2C matrix scan EMI and context-switch noise spikes.
        int maxBin = 0;
        int maxRssi = -128;
        int minRssi = 127;
        if (xHandle) vTaskSuspend(xHandle);
        for (int b = 0; b < WF_BINS; b++) {
            float f = f_start + (f_end - f_start) * b / (WF_BINS - 1);
            setMHZ(f);
            delayMicroseconds(900); // let the PLL/RSSI settle
            int rssi = getMedianRssi();

            rawRssi[b] = rssi;
            if (EscPress) break;
        }
        if (xHandle) vTaskResume(xHandle);
        tft.drawPixel(0, 0, 0); // Keep CC1101/TFT shared SPI happy once per frame before display updates
        delay(2); // yield to allow input handler to process any pending keys

        // Spatial 3-point median filter across frequency bins to completely eliminate
        // isolated 1-bin impulse noise spikes (EMI/clock glitch beating)
        int cleanRssi[WF_BINS];
        for (int b = 0; b < WF_BINS; b++) {
            int prev = (b > 0) ? rawRssi[b - 1] : rawRssi[b];
            int curr = rawRssi[b];
            int next = (b < WF_BINS - 1) ? rawRssi[b + 1] : rawRssi[b];
            int med = curr;
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
            // Adaptive Noise Floor: fast track down, slow drift up
            if ((float)minRssi < agcFloor) {
                agcFloor = agcFloor * 0.7f + (float)minRssi * 0.3f;
            } else {
                agcFloor = agcFloor * 0.96f + (float)minRssi * 0.04f;
            }
            agcFloor = constrain(agcFloor, -115.0f, -50.0f);

            // Adaptive Peak Tracker: fast attack on peaks, smooth decay
            if ((float)maxRssi > agcPeak) {
                agcPeak = agcPeak * 0.6f + (float)maxRssi * 0.4f;
            } else {
                agcPeak = agcPeak * 0.97f + (float)maxRssi * 0.03f;
            }
            agcPeak = constrain(agcPeak, -90.0f, -15.0f);

            // Maintain at least 30 dB dynamic range headroom to prevent thermal noise amplification
            if (agcPeak - agcFloor < 30.0f) {
                agcPeak = agcFloor + 30.0f;
            }
        }

        int floorDbm, peakDbm;
        switch (agcMode) {
            case AGC_GAIN_1X:
                floorDbm = -95;
                peakDbm = -35;
                break;
            case AGC_GAIN_2X:
                floorDbm = -100;
                peakDbm = -50;
                break;
            case AGC_GAIN_3X:
                floorDbm = -105;
                peakDbm = -65;
                break;
            case AGC_AUTO:
            default:
                floorDbm = (int)roundf(agcFloor);
                peakDbm = (int)roundf(agcPeak);
                break;
        }

        for (int b = 0; b < WF_BINS; b++) {
            int v = map(cleanRssi[b], floorDbm, peakDbm, 0, 100);
            v = constrain(v, 0, 100);
            bins[b] = (uint8_t)v;
        }

        // Interpolate the bins across the plot columns for a continuous trace,
        // and peak-hold with slow decay so brief bursts stay visible (as in NRF).
        int maxCol = maxBin * (plotW - 1) / (WF_BINS - 1);
        for (int i = 0; i < plotW; i++) {
            int32_t pos = (int32_t)i * (WF_BINS - 1) * 256 / (plotW - 1);
            int bi = pos >> 8;
            int frac = pos & 0xff;
            if (bi >= WF_BINS - 1) {
                bi = WF_BINS - 2;
                frac = 256;
            }
            int v = bins[bi] + (bins[bi + 1] - bins[bi]) * frac / 256;
            env[i] = (uint8_t)(v < 0 ? 0 : (v > 100 ? 100 : v));
            if (env[i] > envPeak[i]) envPeak[i] = env[i];
            else if (envPeak[i]) envPeak[i]--;

            // Ease the drawn trace toward the measurement so the top glides
            // instead of snapping, matching Jam Detect's animated sweep.
            int d = (int)env[i] - (int)disp[i];
            if (d) disp[i] = (uint8_t)((int)disp[i] + (d > 0 ? max(1, d / 3) : min(-1, d / 3)));
        }

        int hlSpan = plotW / 40;
        plot.trace(disp, envPeak, maxCol - hlSpan, maxCol + hlSpan); // eased trace on top
        plot.pushRow(env); // waterfall shows the true measurement

        if (millis() - lastStatus >= 300) {
            lastStatus = millis();
            float peakFreq = f_start + (f_end - f_start) * maxBin / (WF_BINS - 1);
            const char *modeNames[] = {"AGC", "1x", "2x", "3x"};
            plot.status(String(maxRssi) + "dBm @" + String(peakFreq, 2) + "M [" + modeNames[agcMode] + "] SEL/G:gain");
        }

        // Pan the whole window and refresh the ruler + peak history.
        if (check(UpPress) || check(NextPress)) {
            f_start += step;
            f_end += step;
            memset(envPeak, 0, plotW);
            drawRuler();
            delay(80);
        } else if (check(DownPress) || check(PrevPress)) {
            f_start -= step;
            f_end -= step;
            memset(envPeak, 0, plotW);
            drawRuler();
            delay(80);
        }
    }

    if (xHandle) vTaskResume(xHandle);
    free(env);
    free(disp);
    free(envPeak);
    plot.end();
    returnToMenu = true;
    deinitRfModule();
    delay(10);
}
