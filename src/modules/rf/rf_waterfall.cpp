#include "rf_waterfall.h"
#include "core/mykeyboard.h"
#include "core/spectrum_plot.h"
#include "core/waterfall_input.h"
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
// The band is swept in a modest number of bins (not one per pixel): after each
// hop the RSSI needs time to settle before getRssi() is valid (see
// rf_CC1101_rssi), so sampling every pixel would just read the noise floor. We
// sample WF_BINS points with a real settle and then interpolate the envelope
// across the plot columns for a continuous trace.
#define WF_BINS 48
// Settle after each hop: ~90 us PLL lock plus the RSSI averaging time, which
// grows as the RX filter narrows. Too short returns the previous bin's RSSI
// and creates apparent holes; raise WF_SETTLE_BW_US_KHZ if that shows up.
#define WF_SETTLE_BASE_US 200
#define WF_SETTLE_BW_US_KHZ 270000.0f // settle += this / RXBW[kHz]
#define WF_SETTLE_MAX_US 2000
// Re-run the per-bin VCO calibration this often to follow temperature drift.
#define WF_RECAL_MS 30000
// A waterfall row is committed at this pace rather than once per sweep, so the
// visible history keeps its time span however fast the sweep runs. Sweeps in
// between are max-held into the row so short bursts are never dropped.
#define WF_ROW_MS 200

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
    // The hop settle already gives the PLL, RX chain, and RSSI register time to
    // settle. Taking the maximum of two reads can retain the previous bin's
    // stale high value immediately after a retune or antenna-path change.
    int rssi = ELECHOUSE_cc1101.getRssi();
    // Positive values (especially 127/0 from a failed status read) are not
    // valid dBm readings and must not become full-scale waterfall peaks.
    return (rssi >= 0 || rssi < -127) ? -110 : rssi;
}

// Fast frequency hopping (CC1101 datasheet, "Frequency Hopping and
// Multi-Channel Systems"): every bin is calibrated once and its FSCAL3..1 are
// replayed on each hop. With FS_AUTOCAL left on, every IDLE->RX transition
// re-ran the ~720 us VCO calibration, which dominated the sweep time.
struct WaterfallChannel {
    uint8_t freq[3];  // FREQ2, FREQ1, FREQ0
    uint8_t fscal[3]; // FSCAL3, FSCAL2, FSCAL1
};

// MCSM0 with FS_AUTOCAL disabled; PO_TIMEOUT as in the driver default (0x18).
#define WF_MCSM0_MANUAL_CAL 0x08
#define WF_MCSM0_DEFAULT 0x18

static void waterfallFreqWord(float mhz, uint8_t out[3]) {
    const uint32_t word = (uint32_t)lroundf(mhz * 1000000.0f * 65536.0f / 26000000.0f);
    out[0] = (word >> 16) & 0xff;
    out[1] = (word >> 8) & 0xff;
    out[2] = word & 0xff;
}

static void waitWaterfallIdle(uint32_t timeoutUs) {
    delayMicroseconds(50); // let the strobe leave IDLE before polling for it
    uint32_t t0 = micros();
    while (micros() - t0 < timeoutUs) {
        if ((ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) & 0x1F) == 0x01) return;
        delayMicroseconds(20);
    }
}

static void calibrateWaterfall(WaterfallChannel *ch, float f_start, float f_end) {
    // The hops below write the CC1101 registers directly, which bypasses the
    // board's antenna/filter switching (T-Embed CC1101 SW0/SW1, M5 Cap). Tune
    // once through the Bruce setMHZ() wrapper so the RF path follows the band;
    // a range never straddles a band (clampWaterfallRange), so one path fits
    // every bin, and the wrapper only toggles and settles on an actual change.
    setMHZ((f_start + f_end) * 0.5f);
    ELECHOUSE_cc1101.setSidle();
    for (int b = 0; b < WF_BINS; b++) {
        float f = f_start + (f_end - f_start) * b / (WF_BINS - 1);
        waterfallFreqWord(f, ch[b].freq);
        ELECHOUSE_cc1101.SpiWriteBurstReg(CC1101_FREQ2, ch[b].freq, 3);
        ELECHOUSE_cc1101.SpiStrobe(CC1101_SCAL);
        waitWaterfallIdle(2000);
        ch[b].fscal[0] = ELECHOUSE_cc1101.SpiReadReg(CC1101_FSCAL3);
        ch[b].fscal[1] = ELECHOUSE_cc1101.SpiReadReg(CC1101_FSCAL2);
        ch[b].fscal[2] = ELECHOUSE_cc1101.SpiReadReg(CC1101_FSCAL1);
    }
    ELECHOUSE_cc1101.SetRx();
}

// Frequency registers must only be changed while the synthesizer is idle;
// writing FREQ2/FREQ1/FREQ0 while RX is running can leave the RSSI reading
// associated with the previous bin.
static inline void hopWaterfall(WaterfallChannel &ch) {
    ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
    ELECHOUSE_cc1101.SpiWriteBurstReg(CC1101_FREQ2, ch.freq, 3);
    ELECHOUSE_cc1101.SpiWriteBurstReg(CC1101_FSCAL3, ch.fscal, 3);
    ELECHOUSE_cc1101.SpiStrobe(CC1101_SRX);
}

static uint32_t waterfallSettleUs(float bandwidthKHz) {
    uint32_t us = WF_SETTLE_BASE_US + (uint32_t)(WF_SETTLE_BW_US_KHZ / bandwidthKHz);
    return us > WF_SETTLE_MAX_US ? WF_SETTLE_MAX_US : us;
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

static float waterfallPanStep(float range) {
    if (range > 100) return 10;
    if (range > 10) return 1;
    if (range > 1) return 0.1f;
    if (range > 0.1f) return 0.01f;
    return 0.001f;
}

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
    uint8_t *rowAcc = (uint8_t *)malloc(plotW);  // max of sweeps since the last waterfall row
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
        free(rowAcc);
        plot.end();
        displayError("CC1101 not found!", true);
        return;
    }
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setRxBW(waterfallBandwidths[bandwidthIndex]);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_MCSM0, WF_MCSM0_MANUAL_CAL);
    WaterfallChannel channels[WF_BINS];
    calibrateWaterfall(channels, f_start, f_end);
    uint32_t lastCalibration = millis();
    uint32_t settleUs = waterfallSettleUs(waterfallBandwidths[bandwidthIndex]);

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
    float step = waterfallPanStep(f_end - f_start);

    AgcMode agcMode = AGC_AUTO;
    float agcPeak = -30.0f;
    int rawRssi[WF_BINS];
    uint8_t bins[WF_BINS];

    WaterfallControl selectedControl = WATERFALL_CONTROL_FREQ;
    bool controlLocked = false;
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
                selectedControl = (WaterfallControl)wrapIndex(selectedControl + nav, WATERFALL_CONTROL_COUNT);
                lastStatus = 0;
            } else {
                switch (selectedControl) {
                    case WATERFALL_CONTROL_FREQ: panSteps += nav; break;
                    case WATERFALL_CONTROL_AGC:
                        agcMode = (AgcMode)wrapIndex(agcMode + nav, AGC_MODE_COUNT);
                        agcChanged = true;
                        break;
                    case WATERFALL_CONTROL_ZOOM: zoomSteps += nav; break;
                    default:
                    {
                        // Stop at the narrowest / widest filter instead of wrapping.
                        int newIndex = constrain(bandwidthIndex + nav, 0, waterfallBandwidthCount - 1);
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
            lastStatus = 0; // trigger immediate status display update
        }

        // Pan or zoom the whole window and refresh the ruler + peak history.
        bool rangeChanged = false;
        if (panSteps != 0) {
            int oldBand = getWaterfallBandIndex(f_start);
            bool up = panSteps > 0;
            int moves = up ? panSteps : -panSteps;
            while (moves-- > 0) panWaterfall(f_start, f_end, step, up);
            if (getWaterfallBandIndex(f_start) != oldBand) {
                agcPeak = -50.0f;
                memset(disp, 0, plotW);
            }
            memset(envPeak, 0, plotW);
            rangeChanged = true;
        }
        if (zoomSteps != 0) {
            int moves = zoomSteps > 0 ? zoomSteps : -zoomSteps;
            while (moves-- > 0) {
                int nextZoomIndex = zoomIndex + (zoomSteps > 0 ? -1 : 1);
                if (nextZoomIndex < 0 || nextZoomIndex >= waterfallZoomCount) break;
                zoomIndex = nextZoomIndex;
                setWaterfallZoomRange(f_start, f_end, zoomIndex);
            }
            memset(envPeak, 0, plotW);
            memset(disp, 0, plotW);
            rangeChanged = true;
        }

        if (bandwidthChanged) {
            // RXBW changes are applied in IDLE; the next hop restarts RX with
            // the newly selected filter.
            ELECHOUSE_cc1101.setSidle();
            ELECHOUSE_cc1101.setRxBW(waterfallBandwidths[bandwidthIndex]);
            ELECHOUSE_cc1101.SetRx();
            settleUs = waterfallSettleUs(waterfallBandwidths[bandwidthIndex]);
            lastStatus = 0;
        }
        if (rangeChanged) {
            memset(rowAcc, 0, plotW); // half a row from the old range would be misleading
            lastRow = millis();
            step = waterfallPanStep(f_end - f_start);
            drawRuler();
            lastStatus = 0;
        }
        // Recalibrate for the new bins, and periodically to follow VCO drift.
        if (rangeChanged || millis() - lastCalibration >= WF_RECAL_MS) {
            calibrateWaterfall(channels, f_start, f_end);
            lastCalibration = millis();
        }

        // Sweep the band once — WF_BINS RSSI samples with a real settle so the
        // reading reflects the tuned frequency instead of the noise floor.
        int maxBin = 0;
        int maxRssi = -128;
        int minRssi = 127;
        int sampleCount = 0;
        for (int b = 0; b < WF_BINS; b++) {
            hopWaterfall(channels[b]);
            delayMicroseconds(settleUs);
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
            // Let the input task run between bins; the settle delay alone does
            // not yield. Then collect whatever it reported.
            yield();
            pollWaterfallInput(pending);
            if (pending.esc) break;
        }
        // Esc mid-sweep: drop the partial sweep and handle it at the top.
        if (sampleCount != WF_BINS) continue;

        tft.drawPixel(0, 0, 0); // Keep CC1101/TFT shared SPI happy once per frame before display updates

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
        pollWaterfallInput(pending);
        for (int i = 0; i < plotW; i++)
            if (env[i] > rowAcc[i]) rowAcc[i] = env[i];
        if (millis() - lastRow >= WF_ROW_MS) {
            lastRow = millis();
            plot.pushRow(rowAcc); // waterfall shows the true measurement
            memset(rowAcc, 0, plotW);
        }
        pollWaterfallInput(pending);

        if (millis() - lastStatus >= 300) {
            lastStatus = millis();
            const char *modeNames[] = {"AUTO", "1x", "2x", "3x"};
            String items[WATERFALL_CONTROL_COUNT] = {
                String((f_start + f_end) * 0.5f, 2) + "M",
                "AGC " + String(modeNames[agcMode]),
                "ZOOM " + String(zoomIndex + 1),
                "BW " + String(waterfallBandwidths[bandwidthIndex], 1) + "k",
            };
            String controlBar = waterfallControlBar(items, WATERFALL_CONTROL_COUNT, selectedControl, controlLocked);
            plot.status(controlBar);
        }

        delay(2); // give lower-priority tasks a slice once per frame
    }

    m_rf_waterfall_start_freq = f_start;
    m_rf_waterfall_end_freq = f_end;
    free(env);
    free(disp);
    free(envPeak);
    free(rowAcc);
    bruceConfigPins.rfFxdFreq = previousRfFxdFreq;
    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_MCSM0, WF_MCSM0_DEFAULT);
    deinitRfModule();
    plot.end();
    delay(10);
}
