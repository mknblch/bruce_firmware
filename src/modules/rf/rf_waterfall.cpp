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
    std::vector<String> optionLabels;
    int ind = 0;
    int arraySize = sizeof(subghz_frequency_list) / sizeof(subghz_frequency_list[0]);
    float minDiff = 9999.0f;
    optionLabels.reserve(arraySize);
    options.reserve(arraySize);
    for (int i = 0; i < arraySize; i++) {
        float diff = fabsf(subghz_frequency_list[i] - boundary);
        if (diff < minDiff) {
            minDiff = diff;
            ind = i;
        }
        optionLabels.push_back(String(subghz_frequency_list[i], 2) + "Mhz");
        options.push_back({optionLabels.back().c_str(), [&boundary, i]() { boundary = subghz_frequency_list[i]; }});
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
#define WF_BINS 64
// Allow the PLL, RX path, and RSSI register to settle after every retune.
// A shorter delay can return the previous bin's RSSI and create apparent holes.
#define WF_SETTLE_US 2000

struct WaterfallZoom {
    float spanMHz;
};

// Zoom controls only the frequency span. It is intentionally independent of
// the receiver bandwidth, which is selected separately below.
static const WaterfallZoom waterfallZooms[] = {
    {0.25f}, {0.5f}, {1.0f}, {2.0f}, {4.0f}, {8.0f}, {16.0f}, {32.0f},
};

static const int waterfallZoomCount = sizeof(waterfallZooms) / sizeof(waterfallZooms[0]);

static const float waterfallBandwidths[] = {
    58.0f, 67.7f, 81.3f, 101.6f, 116.1f, 135.4f, 162.5f, 203.1f,
    232.1f, 270.8f, 325.0f, 406.3f, 464.3f, 541.7f, 650.0f, 812.5f,
};

static const int waterfallBandwidthCount = sizeof(waterfallBandwidths) / sizeof(waterfallBandwidths[0]);

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

// Select the nearest hardware-defined zoom level for an existing range.
static int waterfallZoomForSpan(float spanMHz) {
    int best = 0;
    float bestDiff = fabsf(spanMHz - waterfallZooms[0].spanMHz);
    for (int i = 1; i < waterfallZoomCount; i++) {
        float diff = fabsf(spanMHz - waterfallZooms[i].spanMHz);
        if (diff < bestDiff) {
            best = i;
            bestDiff = diff;
        }
    }
    return best;
}

static int waterfallBandwidthForValue(float bandwidthKHz) {
    int best = 0;
    float bestDiff = fabsf(bandwidthKHz - waterfallBandwidths[0]);
    for (int i = 1; i < waterfallBandwidthCount; i++) {
        float diff = fabsf(bandwidthKHz - waterfallBandwidths[i]);
        if (diff < bestDiff) {
            best = i;
            bestDiff = diff;
        }
    }
    return best;
}

static void setWaterfallZoomRange(float &start, float &end, int zoomIndex) {
    zoomIndex = constrain(zoomIndex, 0, waterfallZoomCount - 1);
    float center = (start + end) * 0.5f;
    float span = waterfallZooms[zoomIndex].spanMHz;
    start = center - span * 0.5f;
    end = center + span * 0.5f;
    clampWaterfallRange(start, end);
}

// Frequency registers must only be changed while the synthesizer is idle.
// setMHZ() restarts RX after writing them, which is too late for a sweep:
// writing FREQ2/FREQ1/FREQ0 while RX is running can leave the RSSI reading
// associated with the previous bin, especially after changing RX bandwidth.
static inline void tuneWaterfall(float frequency) {
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(frequency);
    ELECHOUSE_cc1101.SetRx();
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
    clampWaterfallRange(start, end);
}

static inline int sampleRssi() {
    // WF_SETTLE_US already gives the PLL, RX chain, and RSSI register time to
    // settle. Taking the maximum of two reads can retain the previous bin's
    // stale high value immediately after a retune or antenna-path change.
    int rssi = ELECHOUSE_cc1101.getRssi();
    // Positive values (especially 127/0 from a failed status read) are not
    // valid dBm readings and must not become full-scale waterfall peaks.
    return (rssi >= 0 || rssi < -127) ? -110 : rssi;
}

enum AgcMode {
    AGC_AUTO = 0,
    AGC_GAIN_1X, // Normal
    AGC_GAIN_2X, // Boost
    AGC_GAIN_3X, // Max
    AGC_MODE_COUNT
};

enum WaterfallControl {
    WATERFALL_CONTROL_FREQ = 0,
    WATERFALL_CONTROL_AGC,
    WATERFALL_CONTROL_ZOOM,
    WATERFALL_CONTROL_BW,
    WATERFALL_CONTROL_COUNT
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

    const bool previousRfFxdFreq = bruceConfigPins.rfFxdFreq;
    bruceConfigPins.rfFxdFreq = false;
    float f_start = m_rf_waterfall_start_freq;
    float f_end = m_rf_waterfall_end_freq;
    if (f_end < f_start) {
        float t = f_start;
        f_start = f_end;
        f_end = t;
    }
    clampWaterfallRange(f_start, f_end);
    int zoomIndex = waterfallZoomForSpan(f_end - f_start);
    setWaterfallZoomRange(f_start, f_end, zoomIndex);
    int bandwidthIndex = waterfallBandwidthForValue(270.8f);

    if (!initRfModule("rx", f_start)) {
        deinitRfModule();
        bruceConfigPins.rfFxdFreq = previousRfFxdFreq;
        free(env);
        free(disp);
        free(envPeak);
        plot.end();
        displayError("CC1101 not found!", true);
        return;
    }
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setRxBW(waterfallBandwidths[bandwidthIndex]);
    ELECHOUSE_cc1101.SetRx();

    // Show only the band edges and center under the plot; redrawn when navigating.
    const int tickCount = 3;
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
    float agcPeak = -30.0f;
    int rawRssi[WF_BINS];
    uint8_t bins[WF_BINS];

    WaterfallControl selectedControl = WATERFALL_CONTROL_FREQ;
    bool controlLocked = false;
    uint32_t lastStatus = 0;
    while (!returnToMenu) {
        // Select locks the footer control; Esc unlocks it, then exits when
        // the footer is already unlocked.
        if (check(EscPress)) {
            if (controlLocked) {
                controlLocked = false;
                lastStatus = 0;
            } else {
                break;
            }
        }
        bool selectPressed = check(SelPress);

        // Toggle AGC / gain mode from the Cardputer's g/a/s shortcuts.
        bool modeChanged = false;
        bool agcChangedByControl = false;
        bool bandwidthChanged = false;

        bool zoomIn = false;
        bool zoomOut = false;
        bool panUp = false;
        bool panDown = false;
        int32_t panSteps = 0;
        int32_t zoomInSteps = 0;
        int32_t zoomOutSteps = 0;
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
        // Cardputer Enter is reported in the keystroke itself rather than
        // through SelPress. Treat both inputs as the same Select action.
        if (k.enter) selectPressed = true;
        if (selectPressed) {
            controlLocked = !controlLocked;
            lastStatus = 0;
        }

        bool next = false;
        bool previous = false;
#ifdef HAS_ENCODER
        // Rotary movement is accumulated independently of the one-shot button
        // flags. Consume the whole backlog so a sweep cannot make turns appear
        // to be lost. The encoder also raises NextPress/PrevPress for menus;
        // clear those duplicates here because RotaryNetSteps is authoritative.
        int32_t rotarySteps = drainRotarySteps();
        NextPress = false;
        PrevPress = false;
        int32_t nextSteps = rotarySteps < 0 ? -rotarySteps : 0;
        int32_t previousSteps = rotarySteps > 0 ? rotarySteps : 0;
#else
        int32_t nextSteps = check(NextPress) ? 1 : 0;
        int32_t previousSteps = check(PrevPress) ? 1 : 0;
#endif
        if (!controlLocked) {
            while (nextSteps-- > 0) {
                selectedControl = (WaterfallControl)((selectedControl + 1) % WATERFALL_CONTROL_COUNT);
                lastStatus = 0;
            }
            while (previousSteps-- > 0) {
                selectedControl = (WaterfallControl)((selectedControl + WATERFALL_CONTROL_COUNT - 1) % WATERFALL_CONTROL_COUNT);
                lastStatus = 0;
            }
        } else {
            if (selectedControl == WATERFALL_CONTROL_AGC) {
                while (nextSteps-- > 0) {
                    agcMode = (AgcMode)((agcMode + 1) % AGC_MODE_COUNT);
                    modeChanged = true;
                    agcChangedByControl = true;
                }
                while (previousSteps-- > 0) {
                    agcMode = (AgcMode)((agcMode + AGC_MODE_COUNT - 1) % AGC_MODE_COUNT);
                    modeChanged = true;
                    agcChangedByControl = true;
                }
            } else if (selectedControl == WATERFALL_CONTROL_FREQ) {
                panSteps = nextSteps - previousSteps;
                panUp = nextSteps > 0;
                panDown = previousSteps > 0;
            } else if (selectedControl == WATERFALL_CONTROL_ZOOM) {
                zoomInSteps = nextSteps;
                zoomOutSteps = previousSteps;
                zoomIn = nextSteps > 0;
                zoomOut = previousSteps > 0;
            } else {
                while (nextSteps-- > 0) {
                    bandwidthIndex = (bandwidthIndex + 1) % waterfallBandwidthCount;
                    bandwidthChanged = true;
                }
                while (previousSteps-- > 0) {
                    bandwidthIndex = (bandwidthIndex + waterfallBandwidthCount - 1) % waterfallBandwidthCount;
                    bandwidthChanged = true;
                }
                if (bandwidthChanged) lastStatus = 0;
            }
        }

        if (modeChanged && !agcChangedByControl) {
            agcMode = (AgcMode)((agcMode + 1) % AGC_MODE_COUNT);
            memset(envPeak, 0, plotW);
            memset(disp, 0, plotW);
            lastStatus = 0; // trigger immediate status display update
        } else if (agcChangedByControl) {
            memset(envPeak, 0, plotW);
            memset(disp, 0, plotW);
            lastStatus = 0; // trigger immediate status display update
        }

        // Sweep the band once — WF_BINS RSSI samples with a real settle so the
        // reading reflects the tuned frequency instead of the noise floor.
        int maxBin = 0;
        int maxRssi = -128;
        int minRssi = 127;
        int sampleCount = 0;
        for (int b = 0; b < WF_BINS; b++) {
            float f = f_start + (f_end - f_start) * b / (WF_BINS - 1);
            tuneWaterfall(f);
            delayMicroseconds(WF_SETTLE_US);
            int rssi = sampleRssi();

            rawRssi[b] = rssi;
            sampleCount++;
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
        delay(8); // yield briefly for input processing

        if (sampleCount != WF_BINS) break;

        // Adaptive Peak Tracker: fast attack on peaks, responsive decay so sensitivity recovers quickly
        if ((float)maxRssi > agcPeak) {
            agcPeak = agcPeak * 0.5f + (float)maxRssi * 0.5f;
        } else {
            agcPeak = agcPeak * 0.85f + (float)maxRssi * 0.15f;
        }
        agcPeak = constrain(agcPeak, -115.0f, -10.0f);

        // Use the completed sweep's floor so stale AGC state cannot turn valid bins into holes.
        float floorDbm = (float)minRssi;
        float peakDbm;
        switch (agcMode) {
            case AGC_GAIN_1X:
                // Normal / Wide dynamic range: 28 dB span above noise floor
                peakDbm = floorDbm + 28.0f;
                break;
            case AGC_GAIN_2X:
                // Boost / Medium dynamic range: 16 dB span above noise floor
                peakDbm = floorDbm + 16.0f;
                break;
            case AGC_GAIN_3X:
                // Max Sensitivity / Zoom: 8 dB span above noise floor
                peakDbm = floorDbm + 8.0f;
                break;
            case AGC_AUTO:
            default:
                // Adaptive Auto: dynamic contrast with 12 dB min headroom, capped at 25 dB max span
                // so signals always produce vibrant colors and details aren't crushed
                peakDbm = constrain(agcPeak, floorDbm + 12.0f, floorDbm + 25.0f);
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
        int hlStart = max(0, maxCol - hlSpan);
        int hlEnd = min(plotW - 1, maxCol + hlSpan);
        plot.trace(disp, envPeak, hlStart, hlEnd); // eased trace on top
        plot.pushRow(env); // waterfall shows the true measurement

        if (millis() - lastStatus >= 300) {
            lastStatus = millis();
            const char *modeNames[] = {"AUTO", "1x", "2x", "3x"};
            String controlBar;
            for (int i = 0; i < WATERFALL_CONTROL_COUNT; i++) {
                if (i > 0) controlBar += " ";
                if (i == selectedControl) controlBar += controlLocked ? "[" : ">";
                switch (i) {
                    case WATERFALL_CONTROL_FREQ:
                        controlBar += "FREQ " + String((f_start + f_end) * 0.5f, 2);
                        break;
                    case WATERFALL_CONTROL_AGC:
                        controlBar += "AGC " + String(modeNames[agcMode]);
                        break;
                    case WATERFALL_CONTROL_ZOOM:
                        controlBar += "ZOOM Z" + String(zoomIndex + 1);
                        break;
                    case WATERFALL_CONTROL_BW:
                        controlBar += "BW " + String(waterfallBandwidths[bandwidthIndex], 1) + "k";
                        break;
                }
                if (i == selectedControl) controlBar += controlLocked ? "]" : "<";
            }
            plot.status(controlBar);
        }

        // Pan or zoom the whole window and refresh the ruler + peak history.
        bool rangeChanged = false;
        if (check(UpPress) || panUp) {
            int oldBand = getWaterfallBandIndex(f_start);
            int moves = panSteps > 0 ? panSteps : 1;
            while (moves-- > 0) panWaterfall(f_start, f_end, step, true);
            int newBand = getWaterfallBandIndex(f_start);
            if (newBand != oldBand) {
                agcPeak = -50.0f;
                memset(disp, 0, plotW);
            }
            memset(envPeak, 0, plotW);
            drawRuler();
            rangeChanged = true;
            delay(1);
        } else if (check(DownPress) || panDown) {
            int oldBand = getWaterfallBandIndex(f_start);
            int moves = panSteps < 0 ? -panSteps : 1;
            while (moves-- > 0) panWaterfall(f_start, f_end, step, false);
            int newBand = getWaterfallBandIndex(f_start);
            if (newBand != oldBand) {
                agcPeak = -50.0f;
                memset(disp, 0, plotW);
            }
            memset(envPeak, 0, plotW);
            drawRuler();
            rangeChanged = true;
            delay(1);
        } else if (zoomIn || zoomOut) {
            int moves = zoomIn ? max(1, (int)zoomInSteps) : max(1, (int)zoomOutSteps);
            while (moves-- > 0) {
                int nextZoomIndex = zoomIndex + (zoomIn ? -1 : 1);
                if (nextZoomIndex < 0 || nextZoomIndex >= waterfallZoomCount) break;
                zoomIndex = nextZoomIndex;
                setWaterfallZoomRange(f_start, f_end, zoomIndex);
            }
            memset(envPeak, 0, plotW);
            memset(disp, 0, plotW);
            drawRuler();
            rangeChanged = true;
        }

        if (rangeChanged || bandwidthChanged) {
            range = f_end - f_start;
            if (range > 100) step = 10;
            else if (range > 10) step = 1;
            else if (range > 1) step = 0.1f;
            else if (range > 0.1f) step = 0.01f;
            else step = 0.001f;

            if (bandwidthChanged) {
                // RXBW changes are applied in IDLE and RX is restarted so the
                // next sweep uses the newly selected filter.
                ELECHOUSE_cc1101.setSidle();
                ELECHOUSE_cc1101.setRxBW(waterfallBandwidths[bandwidthIndex]);
                ELECHOUSE_cc1101.SetRx();
            }
        }
    }

    m_rf_waterfall_start_freq = f_start;
    m_rf_waterfall_end_freq = f_end;
    free(env);
    free(disp);
    free(envPeak);
    bruceConfigPins.rfFxdFreq = previousRfFxdFreq;
    deinitRfModule();
    plot.end();
    delay(10);
}
