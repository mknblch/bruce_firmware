#include "rf_waterfall.h"
#include "core/mykeyboard.h"
#include "core/spectrum_plot.h"
float m_rf_waterfall_start_freq = 433.0;
float m_rf_waterfall_end_freq = 435.0;

void rf_waterfall() {
    bruceConfigPins.rfFxdFreq = false;
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
        rf_waterfall_boundary_freq(m_rf_waterfall_start_freq, m_rf_waterfall_end_freq, true);
        goto select;
    } else if (option == 2) {
        rf_waterfall_boundary_freq(m_rf_waterfall_end_freq, m_rf_waterfall_start_freq, false);
        goto select;
    }
}

struct WaterfallBand {
    float low;
    float high;
};

static const WaterfallBand waterfallBands[] = {
    {300.0f, 348.0f},
    {387.0f, 464.0f},
    {779.0f, 928.0f},
};

static int getWaterfallBandIndex(float freq) {
    const int bandCount = sizeof(waterfallBands) / sizeof(waterfallBands[0]);
    for (int i = 0; i < bandCount; i++) {
        if (freq >= waterfallBands[i].low && freq <= waterfallBands[i].high) {
            return i;
        }
    }
    int closest = 1;
    float minDist = 9999.0f;
    for (int i = 0; i < bandCount; i++) {
        float dist = 0.0f;
        if (freq < waterfallBands[i].low) dist = waterfallBands[i].low - freq;
        else if (freq > waterfallBands[i].high) dist = freq - waterfallBands[i].high;
        if (dist < minDist) {
            minDist = dist;
            closest = i;
        }
    }
    return closest;
}

void rf_waterfall_boundary_freq(float &boundary, float &otherBoundary, bool isStart) {
    options = {};
    int ind = 0;
    int arraySize = sizeof(subghz_frequency_list) / sizeof(subghz_frequency_list[0]);
    float minDiff = 9999.0f;
    for (int i = 0; i < arraySize; i++) {
        float diff = fabsf(subghz_frequency_list[i] - boundary);
        if (diff < minDiff) {
            minDiff = diff;
            ind = i;
        }
        String tmp = String(subghz_frequency_list[i], 2) + "Mhz";
        options.push_back({tmp.c_str(), [&boundary, i]() { boundary = subghz_frequency_list[i]; }});
    }
    loopOptions(options, ind);
    options.clear();

    int curBand = getWaterfallBandIndex(boundary);
    int otherBand = getWaterfallBandIndex(otherBoundary);
    if (curBand >= 0 && (otherBand != curBand || otherBoundary == boundary)) {
        if (isStart) {
            otherBoundary = min(waterfallBands[curBand].high, boundary + 2.0f);
            if (otherBoundary <= boundary) otherBoundary = boundary + 0.5f;
        } else {
            otherBoundary = max(waterfallBands[curBand].low, boundary - 2.0f);
            if (otherBoundary >= boundary) otherBoundary = boundary - 0.5f;
        }
    }
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
#define WF_BINS 32

// Keep the entire scan inside one CC1101-supported band while preserving span.
static void clampWaterfallRange(float &start, float &end) {
    if (end < start) {
        float t = start;
        start = end;
        end = t;
    }
    float span = end - start;
    if (span < 0.05f) span = 2.0f;

    int bandIdx = getWaterfallBandIndex(start);
    const WaterfallBand &band = waterfallBands[bandIdx];

    if (span > (band.high - band.low)) {
        span = band.high - band.low;
    }

    if (start < band.low) {
        start = band.low;
        end = start + span;
    } else if (start + span > band.high) {
        end = band.high;
        start = end - span;
    } else {
        end = start + span;
    }
}

static void panWaterfall(float &start, float &end, float step, bool up) {
    float span = end - start;
    if (span < 0.05f) span = 2.0f;

    int bandIdx = getWaterfallBandIndex(start);
    const int bandCount = sizeof(waterfallBands) / sizeof(waterfallBands[0]);

    if (up) {
        start += step;
        end = start + span;
        if (end > waterfallBands[bandIdx].high) {
            if (bandIdx + 1 < bandCount) {
                // Transition to next higher band
                start = waterfallBands[bandIdx + 1].low;
                end = start + span;
                if (end > waterfallBands[bandIdx + 1].high) {
                    end = waterfallBands[bandIdx + 1].high;
                    start = end - span;
                }
            } else {
                end = waterfallBands[bandIdx].high;
                start = end - span;
            }
        }
    } else {
        start -= step;
        end = start + span;
        if (start < waterfallBands[bandIdx].low) {
            if (bandIdx > 0) {
                // Transition to next lower band
                end = waterfallBands[bandIdx - 1].high;
                start = end - span;
                if (start < waterfallBands[bandIdx - 1].low) {
                    start = waterfallBands[bandIdx - 1].low;
                    end = start + span;
                }
            } else {
                start = waterfallBands[bandIdx].low;
                end = start + span;
            }
        }
    }
}

static inline int samplePeakRssi() {
    int r1 = ELECHOUSE_cc1101.getRssi();
    delayMicroseconds(60);
    int r2 = ELECHOUSE_cc1101.getRssi();
    return (r1 > r2) ? r1 : r2;
}

enum AgcMode {
    AGC_AUTO = 0,
    AGC_GAIN_1X, // Normal
    AGC_GAIN_2X, // Boost
    AGC_GAIN_3X, // Max
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

    bruceConfigPins.rfFxdFreq = false;
    float f_start = m_rf_waterfall_start_freq;
    float f_end = m_rf_waterfall_end_freq;
    if (f_end < f_start) {
        float t = f_start;
        f_start = f_end;
        f_end = t;
    }
    clampWaterfallRange(f_start, f_end);

    initRfModule("rx", f_start);
    ELECHOUSE_cc1101.setRxBW(256);

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
    float agcFloor = -110.0f;
    float agcPeak = -30.0f;
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
            lastStatus = 0; // trigger immediate status display update
        }

        // Sweep the band once — WF_BINS RSSI samples with a real settle so the
        // reading reflects the tuned frequency instead of the noise floor.
        int maxBin = 0;
        int maxRssi = -128;
        int minRssi = 127;
        for (int b = 0; b < WF_BINS; b++) {
            float f = f_start + (f_end - f_start) * b / (WF_BINS - 1);
            setMHZ(f);
            delayMicroseconds(850); // let the PLL/RSSI settle
            int rssi = samplePeakRssi();

            rawRssi[b] = rssi;
            if (rssi > maxRssi) {
                maxRssi = rssi;
                maxBin = b;
            }
            if (rssi < minRssi) {
                minRssi = rssi;
            }
            if (EscPress) break;
        }
        tft.drawPixel(0, 0, 0); // Keep CC1101/TFT shared SPI happy once per frame before display updates
        delay(2); // yield briefly for input processing

        // Adaptive Noise Floor: fast track down, slow drift up
        if ((float)minRssi < agcFloor) {
            agcFloor = agcFloor * 0.7f + (float)minRssi * 0.3f;
        } else {
            agcFloor = agcFloor * 0.95f + (float)minRssi * 0.05f;
        }
        agcFloor = constrain(agcFloor, -125.0f, -40.0f);

        // Adaptive Peak Tracker: fast attack on peaks, responsive decay so sensitivity recovers quickly
        if ((float)maxRssi > agcPeak) {
            agcPeak = agcPeak * 0.5f + (float)maxRssi * 0.5f;
        } else {
            agcPeak = agcPeak * 0.85f + (float)maxRssi * 0.15f;
        }
        agcPeak = constrain(agcPeak, -115.0f, -10.0f);

        float floorDbm = agcFloor;
        float peakDbm;
        switch (agcMode) {
            case AGC_GAIN_1X:
                // Normal / Wide dynamic range: 28 dB span above noise floor
                peakDbm = agcFloor + 28.0f;
                break;
            case AGC_GAIN_2X:
                // Boost / Medium dynamic range: 16 dB span above noise floor
                peakDbm = agcFloor + 16.0f;
                break;
            case AGC_GAIN_3X:
                // Max Sensitivity / Zoom: 8 dB span above noise floor
                peakDbm = agcFloor + 8.0f;
                break;
            case AGC_AUTO:
            default:
                // Adaptive Auto: dynamic contrast with 12 dB min headroom, capped at 25 dB max span
                // so signals always produce vibrant colors and details aren't crushed
                peakDbm = constrain(agcPeak, agcFloor + 12.0f, agcFloor + 25.0f);
                break;
        }

        float span = max(6.0f, peakDbm - floorDbm);
        for (int b = 0; b < WF_BINS; b++) {
            float norm = ((float)rawRssi[b] - floorDbm) / span;
            int v = (int)roundf(norm * 100.0f);
            bins[b] = (uint8_t)constrain(v, 0, 100);
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
            env[i] = (uint8_t)constrain(v, 0, 100);

            if (env[i] >= envPeak[i]) {
                envPeak[i] = env[i];
            } else if (envPeak[i] > 0) {
                envPeak[i]--;
            }

            // Fast attack on bursts so short transmissions hit full trace height, smooth decay on drop
            if (env[i] > disp[i]) {
                disp[i] = env[i];
            } else if (disp[i] > env[i]) {
                int drop = (disp[i] - env[i] > 6) ? 6 : (disp[i] - env[i]);
                disp[i] -= drop;
            }
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
            int oldBand = getWaterfallBandIndex(f_start);
            panWaterfall(f_start, f_end, step, true);
            int newBand = getWaterfallBandIndex(f_start);
            if (newBand != oldBand) {
                agcFloor = -110.0f;
                agcPeak = -50.0f;
                memset(disp, 0, plotW);
            }
            memset(envPeak, 0, plotW);
            drawRuler();
            delay(80);
        } else if (check(DownPress) || check(PrevPress)) {
            int oldBand = getWaterfallBandIndex(f_start);
            panWaterfall(f_start, f_end, step, false);
            int newBand = getWaterfallBandIndex(f_start);
            if (newBand != oldBand) {
                agcFloor = -110.0f;
                agcPeak = -50.0f;
                memset(disp, 0, plotW);
            }
            memset(envPeak, 0, plotW);
            drawRuler();
            delay(80);
        }
    }

    m_rf_waterfall_start_freq = f_start;
    m_rf_waterfall_end_freq = f_end;
    free(env);
    free(disp);
    free(envPeak);
    plot.end();
    returnToMenu = true;
    deinitRfModule();
    delay(10);
}
