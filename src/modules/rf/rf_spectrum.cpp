#include "rf_spectrum.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/spectrum_plot.h"
#include "protocols/rf_config.h"
#include "protocols/rf_decoder.h"
#include "protocols/rf_encoder.h"
#include "rf_utils.h"
#include "save.h"
#include "structs.h"

// Plot band, derived from the panel so the graph never collides with the title
// bar or the status line on any of the supported screens.
static inline int rf_plot_top() { return BORDER_PAD_Y + 8 * FM + 2; }
static inline int rf_plot_bot() { return tftHeight - 8 * FP - 8; }
// Side margins keep the graph clear of the rounded theme border at x = 5.
static inline int rf_plot_left() { return 8; }
static inline int rf_plot_width() { return tftWidth - 16; }
static inline uint16_t rf_grid_color() {
    return blendColors(bruceConfig.bgColor, bruceConfig.priColor, 55);
}
static inline uint16_t rf_label_color() {
    return blendColors(bruceConfig.priColor, TFT_WHITE, 120);
}

// Frame plus a single status line at the bottom. Drawn once, and again only
// when the tuning changes, so the live graph never flickers.
static void draw_rf_header(const String &title, const String &info) {
    drawMainBorderWithTitle(title);
    tft.setTextSize(FP);
    tft.fillRect(rf_plot_left(), rf_plot_bot() + 2, rf_plot_width(), 8 * FP, bruceConfig.bgColor);
    tft.setTextColor(rf_label_color(), bruceConfig.bgColor);
    tft.drawString(info, rf_plot_left(), rf_plot_bot() + 2, 1);
}

static void rf_envelope(const uint8_t *lvl, size_t numChannels, uint8_t *env, int plotW) {
    if (numChannels < 2) {
        memset(env, numChannels ? lvl[0] : 0, plotW);
        return;
    }
    for (int i = 0; i < plotW; i++) {
        int32_t pos = (int32_t)i * (numChannels - 1) * 256 / (plotW - 1);
        int ci = pos >> 8;
        int frac = pos & 0xff;
        if (ci >= (int)numChannels - 1) {
            ci = numChannels - 2;
            frac = 256;
        }
        int v = lvl[ci] + (lvl[ci + 1] - lvl[ci]) * frac / 256;
        env[i] = (uint8_t)(v < 0 ? 0 : (v > 100 ? 100 : v));
    }
}

static void rf_spectrum_range_selection() {
    options = {
        {subghz_frequency_ranges[0], [=]() { bruceConfigPins.setRfScanRange(0); }},
        {subghz_frequency_ranges[1], [=]() { bruceConfigPins.setRfScanRange(1); }},
        {subghz_frequency_ranges[2], [=]() { bruceConfigPins.setRfScanRange(2); }},
        {subghz_frequency_ranges[3], [=]() { bruceConfigPins.setRfScanRange(3); }},
    };
    int idx = constrain(bruceConfigPins.rfScanRange, 0, 3);
    loopOptions(options, idx);
    options.clear();
    bruceConfigPins.rfFxdFreq = false;
}

void rf_spectrum() {
    bruceConfigPins.rfFxdFreq = false;
    if (!initRfModule("rx", bruceConfigPins.rfFreq)) {
        displayError("Error starting RF", true);
        return;
    }

    SpectrumPlot plot;
    if (!plot.begin("RF Spectrum")) {
        displayError("Out of memory", true);
        deinitRfModule();
        return;
    }

    const int plotW = plot.width();
    uint8_t *env = (uint8_t *)malloc(plotW);
    uint8_t *envPeak = (uint8_t *)malloc(plotW);
    if (!env || !envPeak) {
        free(env);
        free(envPeak);
        plot.end();
        deinitRfModule();
        displayError("Out of memory", true);
        return;
    }

    auto updateRuler = [&]() {
        int startIdx = range_limits[bruceConfigPins.rfScanRange][0];
        int endIdx = range_limits[bruceConfigPins.rfScanRange][1];
        int numFreqs = endIdx - startIdx + 1;
        const int tickCount = min(5, numFreqs);
        int cols[5];
        String labels[5];
        for (int i = 0; i < tickCount; i++) {
            int idx = i * (numFreqs - 1) / (tickCount > 1 ? tickCount - 1 : 1);
            cols[i] = idx * (plotW - 1) / (numFreqs > 1 ? numFreqs - 1 : 1);
            labels[i] = String(subghz_frequency_list[startIdx + idx], 1);
        }
        plot.ruler(cols, labels, tickCount);
    };

    updateRuler();

    uint8_t channelLvl[128] = {0};
    uint8_t channelPeak[128] = {0};
    uint32_t lastFrame = 0, lastRow = 0;
    int multiplier = 2;

    while (1) {
        if (check(EscPress)) { break; }

        if (check(SelPress)) {
            rf_spectrum_range_selection();
            plot.redraw("RF Spectrum");
            updateRuler();
            memset(channelLvl, 0, sizeof(channelLvl));
            memset(channelPeak, 0, sizeof(channelPeak));
            continue;
        }

        keyStroke k = _getKeyPress();
        if (k.pressed || !k.word.empty()) {
            for (auto ch : k.word) {
                char lowerKey = tolower(ch);
                if (lowerKey == 'g') {
                    multiplier = (multiplier >= 5) ? 1 : multiplier + 1;
                    memset(channelLvl, 0, sizeof(channelLvl));
                    memset(channelPeak, 0, sizeof(channelPeak));
                }
            }
        }

        int startIdx = range_limits[bruceConfigPins.rfScanRange][0];
        int endIdx = range_limits[bruceConfigPins.rfScanRange][1];
        int numFreqs = endIdx - startIdx + 1;
        if (numFreqs > 128) numFreqs = 128;

        int maxIdx = 0;
        int maxRssiDbm = -120;

        for (int i = 0; i < numFreqs; i++) {
            if (EscPress || SelPress || AnyKeyPress) break;
            float freq = subghz_frequency_list[startIdx + i];
            setMHZ(freq);
            delayMicroseconds(900);
            int rssi = ELECHOUSE_cc1101.getRssi();
            tft.drawPixel(0, 0, 0);

            int rawPct = (rssi <= -88) ? 0 : map(constrain(rssi, -88, -25), -88, -25, 0, 100);
            rawPct = constrain(rawPct * multiplier, 0, 100);

            if (rawPct > channelLvl[i]) {
                channelLvl[i] = (uint8_t)rawPct;
            } else {
                channelLvl[i] = (uint8_t)((channelLvl[i] * 3 + rawPct) / 4);
            }

            if (channelLvl[i] > channelPeak[i]) channelPeak[i] = channelLvl[i];

            if (rssi > maxRssiDbm) {
                maxRssiDbm = rssi;
                maxIdx = i;
            }
        }

        if (millis() - lastFrame >= 40) {
            lastFrame = millis();
            for (int i = 0; i < numFreqs; i++) {
                if (channelPeak[i] > channelLvl[i]) channelPeak[i]--;
            }
            rf_envelope(channelLvl, numFreqs, env, plotW);
            rf_envelope(channelPeak, numFreqs, envPeak, plotW);

            int hlC = maxIdx * (plotW - 1) / (numFreqs > 1 ? numFreqs - 1 : 1);
            int hlSpan = max(2, (2 * (plotW - 1)) / (numFreqs > 1 ? numFreqs - 1 : 1));
            plot.trace(env, envPeak, hlC - hlSpan, hlC + hlSpan);

            if (millis() - lastRow >= 120) {
                lastRow = millis();
                plot.pushRow(env);
                float peakFreq = subghz_frequency_list[startIdx + maxIdx];
                plot.status(
                    "pk: " + String(peakFreq, 2) + "M " + String(maxRssiDbm) + "dBm " +
                    String(multiplier) + "x [" +
                    String(subghz_frequency_ranges[bruceConfigPins.rfScanRange]) + "]"
                );
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    free(env);
    free(envPeak);
    plot.end();
    returnToMenu = true;
    deinitRfModule();
}

static String format_timebase(uint32_t usPerPx) {
    if (usPerPx >= 1000) {
        if (usPerPx % 1000 == 0) return String(usPerPx / 1000) + "ms/px";
        return String(usPerPx / 1000.0, 1) + "ms/px";
    }
    return String(usPerPx) + "us/px";
}

struct SensiLevel {
    const char *label;
    int minTransitions;
};

static const SensiLevel sensiLevels[] = {
    { "Hi", 4 },    // High sensitivity: catches any burst (4+ transitions)
    { "Norm", 10 }, // Normal: standard balanced sensitivity (10+ transitions)
    { "Med", 18 },  // Medium: filters noise spikes (18+ transitions)
    { "Low", 30 }   // Low: strict, multi-byte / long frame transmissions only (30+ transitions)
};
static const size_t numSensi = sizeof(sensiLevels) / sizeof(sensiLevels[0]);

static void render_rf_squarewave(
    const std::vector<int> &durations,
    uint32_t usPerPx,
    int32_t offsetUs,
    bool isHeld,
    bool holdNext,
    const char *sensiLabel,
    bool showInitialPrompt
) {
    const int top = rf_plot_top();
    const int bot = rf_plot_bot();
    const int left = rf_plot_left();
    const int width = rf_plot_width();
    const int right = left + width;

    const int traceH = 7;
    const int rowGap = 5;
    const int rowPitch = traceH + rowGap;
    const int numRows = (bot - top) / rowPitch;

    // Clear plot area
    tft.fillRect(left, top, width, bot - top, bruceConfig.bgColor);

    // Draw baseline guides for each row
    const uint16_t gridColor = rf_grid_color();
    for (int r = 0; r < numRows; r++) {
        int rowY = top + r * rowPitch;
        tft.drawFastHLine(left, rowY + traceH, width, gridColor);
    }

    // Compose bottom info line
    String info = String(bruceConfigPins.rfFreq, 2) + "M " + format_timebase(usPerPx);
    if (offsetUs > 0) {
        info += " +" + (offsetUs >= 1000 ? String(offsetUs / 1000.0, 1) + "ms" : String(offsetUs) + "us");
    }
    info += " " + String(sensiLabel);

    if (durations.empty()) {
        if (showInitialPrompt) {
            tft.setTextSize(FP);
            tft.setTextColor(rf_label_color(), bruceConfig.bgColor);
            String prompt = holdNext ? "Waiting for signal [HOLD]..." : "Waiting for RF signal...";
            int tw = prompt.length() * 6 * FP;
            int tx = left + (width - tw) / 2;
            int ty = top + (bot - top) / 2 - 4;
            tft.drawString(prompt, tx > left ? tx : left, ty, 1);
        }
    } else {
        uint32_t totalUs = 0;
        for (int d : durations) totalUs += abs(d);

        RfCodes code;
        bool decoded = rf_decode_ook(durations, code);
        if (!decoded) decoded = rf_decode_keeloq(durations, code);

        if (decoded) {
            info += " | " + code.protocol + " 0x" + String((uint32_t)code.key, HEX);
            if (code.Bit > 0) info += " (" + String(code.Bit) + "b)";
        } else {
            info += " | " + String(durations.size()) + "e " + String(totalUs / 1000) + "ms";
        }
    }

    if (isHeld) {
        info += " [HOLD]";
    } else if (holdNext) {
        info += " [ARMED]";
    }

    // Draw bottom status bar
    tft.setTextSize(FP);
    tft.fillRect(left, bot + 2, width, 8 * FP, bruceConfig.bgColor);
    tft.setTextColor(rf_label_color(), bruceConfig.bgColor);
    tft.drawString(info, left, bot + 2, 1);

    if (durations.empty() || numRows <= 0) return;

    // Draw waveform trace with offset translation
    int curRow = 0;
    int curX = left;
    int curLevel = -1; // -1: uninitialized, 0: LOW, 1: HIGH
    uint32_t remainderUs = 0;
    uint32_t elapsedUs = 0;

    for (int dur : durations) {
        if (dur == 0) continue;
        int level = (dur > 0) ? 1 : 0;
        uint32_t us = (uint32_t)abs(dur);

        if (elapsedUs + us <= (uint32_t)offsetUs) {
            elapsedUs += us;
            curLevel = level;
            continue;
        }

        uint32_t visibleUs = us;
        if (elapsedUs < (uint32_t)offsetUs) {
            visibleUs = (elapsedUs + us) - (uint32_t)offsetUs;
            elapsedUs = offsetUs;
        } else {
            elapsedUs += us;
        }

        int rowY = top + curRow * rowPitch;

        // If level changed, draw vertical transition edge
        if (curLevel != -1 && curLevel != level) {
            tft.drawFastVLine(curX, rowY, traceH + 1, bruceConfig.priColor);
        }
        curLevel = level;

        // Calculate pixel length with timing remainder accumulator
        uint32_t totalDur = visibleUs + remainderUs;
        int px = totalDur / usPerPx;
        remainderUs = totalDur % usPerPx;
        if (px == 0 && visibleUs > 0) px = 1;

        while (px > 0 && curRow < numRows) {
            rowY = top + curRow * rowPitch;
            int y = (curLevel == 1) ? rowY : (rowY + traceH);
            int spaceLeft = right - curX;

            if (px <= spaceLeft) {
                if (px > 0) {
                    tft.drawFastHLine(curX, y, px, bruceConfig.priColor);
                    curX += px;
                    if (curX >= right) {
                        curRow++;
                        curX = left;
                    }
                }
                px = 0;
            } else {
                if (spaceLeft > 0) {
                    tft.drawFastHLine(curX, y, spaceLeft, bruceConfig.priColor);
                }
                px -= spaceLeft;
                curRow++;
                curX = left;
            }
        }

        if (curRow >= numRows) break;
    }
}

//@Pirata
void rf_SquareWave() {
    bruceConfigPins.setRfFreq(bruceConfigPins.rfFreq, 1);
    if (!initRfModule("rx", bruceConfigPins.rfFreq)) return;
    if (bruceConfigPins.rfModule == CC1101_SPI_MODULE) {
        ELECHOUSE_cc1101.setDcFilterOff(true);
    }

    RfRxSession rx;
    rx.setIdleTimeout(12000000); // 12ms for snappy real-time visualizer updates
    if (!rx.begin()) {
        deinitRfModule();
        return;
    }

    const uint32_t zoomLevels[] = { 1, 2, 5, 10, 20, 50, 100, 250, 500, 1000, 2500, 5000, 10000 };
    const size_t numZoom = sizeof(zoomLevels) / sizeof(zoomLevels[0]);
    size_t zoomIdx = 5; // default 50 us/px
    int32_t offsetUs = 0;
    size_t sensiIdx = 0; // default Hi (maximum sensitivity)
    rx.setSensitivity(sensiLevels[sensiIdx].minTransitions);
    bool isHeld = false;
    bool holdNext = false;
    std::vector<int> durations;
    std::vector<int> lastDurations;

    const int numFreqs = sizeof(subghz_frequency_list) / sizeof(subghz_frequency_list[0]);
    int freqIdx = 0;
    for (int i = 0; i < numFreqs; i++) {
        if (fabs(subghz_frequency_list[i] - bruceConfigPins.rfFreq) < 0.001f) {
            freqIdx = i;
            break;
        }
    }

    tft.drawPixel(0, 0, 0);
    draw_rf_header("RF SquareWave", String(bruceConfigPins.rfFreq, 2) + " MHz");
    render_rf_squarewave(
        lastDurations,
        zoomLevels[zoomIdx],
        offsetUs,
        isHeld,
        holdNext,
        sensiLevels[sensiIdx].label,
        lastDurations.empty()
    );

    auto changeFreq = [&](int delta) {
        freqIdx = (freqIdx + delta + numFreqs) % numFreqs;
        float newFreq = subghz_frequency_list[freqIdx];
        bruceConfigPins.setRfFreq(newFreq, 1);
        setMHZ(newFreq);
        if (bruceConfigPins.rfModule == CC1101_SPI_MODULE) {
            ELECHOUSE_cc1101.setDcFilterOff(true);
        }
        rx.flush();
        lastDurations.clear();
        offsetUs = 0;
        if (isHeld) {
            isHeld = false;
            holdNext = false;
        }
        tft.drawPixel(0, 0, 0);
        draw_rf_header("RF SquareWave", String(bruceConfigPins.rfFreq, 2) + " MHz");
        return true;
    };

    auto zoomWaveform = [&](int delta) {
        if (delta < 0) { // Zoom In
            if (zoomIdx > 0) {
                zoomIdx--;
                return true;
            }
        } else if (delta > 0) { // Zoom Out
            if (zoomIdx + 1 < numZoom) {
                zoomIdx++;
                return true;
            }
        }
        return false;
    };

    auto translateWaveform = [&](int dir) {
        if (lastDurations.empty()) return false;
        uint32_t totalUs = 0;
        for (int d : lastDurations) totalUs += abs(d);
        int32_t stepUs = (int32_t)(rf_plot_width() * zoomLevels[zoomIdx] / 4);
        if (stepUs < 1) stepUs = 1;

        if (dir < 0) { // Left (earlier in time)
            if (offsetUs > 0) {
                offsetUs = max((int32_t)0, offsetUs - stepUs);
                return true;
            }
        } else if (dir > 0) { // Right (later in time)
            if (offsetUs + stepUs < (int32_t)totalUs) {
                offsetUs += stepUs;
                return true;
            }
        }
        return false;
    };

    while (1) {
        bool reRender = false;

        if (!isHeld && rx.poll(durations)) {
            if (!durations.empty()) {
                lastDurations = durations;
                offsetUs = 0;
                if (holdNext) {
                    isHeld = true;
                    holdNext = false;
                }
                reRender = true;
            }
        }

        if (check(EscPress)) { break; }

        if (check(UpPress)) {
            if (zoomWaveform(-1)) reRender = true;
        } else if (check(DownPress)) {
            if (zoomWaveform(+1)) reRender = true;
        }

        if (check(PrevPagePress)) {
            if (translateWaveform(-1)) reRender = true;
        } else if (check(NextPagePress)) {
            if (translateWaveform(+1)) reRender = true;
        } else if (check(PrevPress)) {
            if (translateWaveform(-1)) reRender = true;
        } else if (check(NextPress)) {
            if (translateWaveform(+1)) reRender = true;
        }

        if (setMHZMenu()) {
            for (int i = 0; i < numFreqs; i++) {
                if (fabs(subghz_frequency_list[i] - bruceConfigPins.rfFreq) < 0.001f) {
                    freqIdx = i;
                    break;
                }
            }
            if (bruceConfigPins.rfModule == CC1101_SPI_MODULE) {
                ELECHOUSE_cc1101.setDcFilterOff(true);
            }
            rx.flush();
            lastDurations.clear();
            offsetUs = 0;
            isHeld = false;
            holdNext = false;
            tft.drawPixel(0, 0, 0);
            draw_rf_header("RF SquareWave", String(bruceConfigPins.rfFreq, 2) + " MHz");
            reRender = true;
        }

        keyStroke k = _getKeyPress();
        if (k.pressed || !k.word.empty()) {
            for (auto i : k.word) {
                char lowerKey = tolower((char)i);
                if (i == '[' || i == '{') {
                    if (changeFreq(-1)) reRender = true;
                } else if (i == ']' || i == '}') {
                    if (changeFreq(+1)) reRender = true;
                } else if (i == ';' || (uint8_t)i == 0xDA || i == '+' || i == '=') {
                    if (zoomWaveform(-1)) reRender = true;
                } else if (i == '.' || (uint8_t)i == 0xD9 || i == '-' || i == '_') {
                    if (zoomWaveform(+1)) reRender = true;
                } else if (i == ',' || (uint8_t)i == 0xD8) {
                    if (translateWaveform(-1)) reRender = true;
                } else if (i == '/' || (uint8_t)i == 0xD7) {
                    if (translateWaveform(+1)) reRender = true;
                } else if (lowerKey == 'h' || lowerKey == 'p') {
                    if (isHeld) {
                        isHeld = false;
                        holdNext = false;
                    } else if (holdNext) {
                        holdNext = false;
                    } else if (!lastDurations.empty()) {
                        isHeld = true;
                    } else {
                        holdNext = true;
                    }
                    reRender = true;
                } else if (lowerKey == 's') {
                    if (lastDurations.empty()) {
                        displayWarning("No signal to save!", false);
                        delay(700);
                    } else {
                        String savedFile;
                        if (rf_raw_save_durations(lastDurations, bruceConfigPins.rfFreq, &savedFile)) {
                            delay(1200);
                        }
                    }
                    draw_rf_header("RF SquareWave", String(bruceConfigPins.rfFreq, 2) + " MHz");
                    reRender = true;
                } else if (lowerKey == 'r') {
                    if (lastDurations.empty()) {
                        displayWarning("No signal to replay!", false);
                        delay(700);
                    } else {
                        displayTextLine("Replaying...");
                        rx.end();
                        initRfModule("tx", bruceConfigPins.rfFreq);
                        rf_tx_durations(lastDurations);
                        deinitRfModule();
                        rx.begin();
                        rx.setSensitivity(sensiLevels[sensiIdx].minTransitions);
                        if (bruceConfigPins.rfModule == CC1101_SPI_MODULE) {
                            ELECHOUSE_cc1101.setDcFilterOff(true);
                        }
                        rx.flush();
                        displayTextLine("Replayed!");
                        delay(400);
                    }
                    draw_rf_header("RF SquareWave", String(bruceConfigPins.rfFreq, 2) + " MHz");
                    reRender = true;
                } else if (lowerKey == 'c') {
                    lastDurations.clear();
                    offsetUs = 0;
                    if (isHeld) {
                        isHeld = false;
                        holdNext = true;
                    }
                    rx.flush();
                    reRender = true;
                } else if (lowerKey == 'g' || lowerKey == 't') {
                    sensiIdx = (sensiIdx + 1) % numSensi;
                    rx.setSensitivity(sensiLevels[sensiIdx].minTransitions);
                    reRender = true;
                } else if (i == ' ') {
                    isHeld = false;
                    holdNext = false;
                    reRender = true;
                }
            }
        }

        if (reRender) {
            render_rf_squarewave(
                lastDurations,
                zoomLevels[zoomIdx],
                offsetUs,
                isHeld,
                holdNext,
                sensiLevels[sensiIdx].label,
                lastDurations.empty()
            );
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
    rx.end();
    returnToMenu = true;
    deinitRfModule();
}

void rf_CC1101_rssi() {
#if !defined(LITE_VERSION)
    if (bruceConfigPins.rfModule != CC1101_SPI_MODULE) {
        displayError("only for CC1101 module", true);
        return;
    }
    // Left gutter wide enough for the "-95" scale labels, band derived from the
    // panel instead of assuming a 120px tall screen.
    const int top = rf_plot_top();
    const int bot = rf_plot_bot();
    const int axisX = rf_plot_left() + 3 * FP * LW + 2;
    const int graph_size = rf_plot_left() + rf_plot_width() - axisX - 2;
    std::vector<int> signal(graph_size, -95);
    const size_t freq_count = sizeof(subghz_frequency_list) / sizeof(float);
    std::vector<int> bar_size(freq_count, 0);
    const int max_bar_size = bot - top;
    bool redraw = true;
    int multiplier = 2;
    const int min_value = map(-70, -95, -20, 0, max_bar_size);
    // dBm -> screen row (static grid axis)
    auto axisY = [&](int dbm) { return (int)map(constrain(dbm, -95, -20), -95, -20, bot, top); };
    // dBm -> screen row scaled with multiplier for fixed frequency signal line
    auto rssiY = [&](int rssi) {
        int h = map(constrain(rssi, -95, -20), -95, -20, 0, max_bar_size);
        return bot - constrain(h * multiplier, 0, max_bar_size);
    };

    while (1) {
        if (check(EscPress)) { break; }
        if (check(SelPress)) {
            deinitRfModule();
            rf_range_selection(bruceConfigPins.rfFreq);
            redraw = true;
        }

        keyStroke k = _getKeyPress();
        if (k.pressed || !k.word.empty()) {
            for (auto ch : k.word) {
                char lowerKey = tolower(ch);
                if (lowerKey == 'g') {
                    multiplier = (multiplier >= 5) ? 1 : multiplier + 1;
                    if (bruceConfigPins.rfFxdFreq) {
                        draw_rf_header("RF RSSI " + String(multiplier) + "x", String(bruceConfigPins.rfFreq, 2) + " MHz");
                        std::fill(signal.begin(), signal.end(), -95);
                        tft.fillRect(rf_plot_left(), top, rf_plot_width(), bot - top, bruceConfig.bgColor);
                        tft.drawFastVLine(axisX, top, bot - top, bruceConfig.priColor);
                        tft.setTextColor(rf_label_color(), bruceConfig.bgColor);
                        for (int dbm = -95; dbm <= -20; dbm += 15) {
                            int y = axisY(dbm) - (8 * FP) / 2;
                            tft.drawString(String(dbm), 8, y, 1);
                            tft.drawFastHLine(axisX - 2, axisY(dbm), 3, rf_grid_color());
                        }
                    } else {
                        draw_rf_header(
                            String("RF RSSI ") + subghz_frequency_ranges[bruceConfigPins.rfScanRange] + " " +
                                String(multiplier) + "x",
                            ""
                        );
                        std::fill(bar_size.begin(), bar_size.end(), 0);
                        tft.fillRect(rf_plot_left(), top, rf_plot_width(), bot - top, bruceConfig.bgColor);
                    }
                }
            }
        }

        if (redraw) {
            redraw = false;
            tft.drawPixel(0, 0, 0);
            tft.setTextSize(FP);
            // Fixed frequency sees a dot running grafic, showing RSSI over time
            if (bruceConfigPins.rfFxdFreq) {
                if (!initRfModule("rx", bruceConfigPins.rfFreq))
                    displayError("Error setting frequency", true);
                draw_rf_header("RF RSSI " + String(multiplier) + "x", String(bruceConfigPins.rfFreq, 2) + " MHz");
                tft.fillRect(rf_plot_left(), top, rf_plot_width(), bot - top, bruceConfig.bgColor);
                tft.drawFastVLine(axisX, top, bot - top, bruceConfig.priColor);
                tft.setTextColor(rf_label_color(), bruceConfig.bgColor);
                for (int dbm = -95; dbm <= -20; dbm += 15) {
                    int y = axisY(dbm) - (8 * FP) / 2;
                    tft.drawString(String(dbm), 8, y, 1);
                    tft.drawFastHLine(axisX - 2, axisY(dbm), 3, rf_grid_color());
                }
                // resets signal array
                std::fill(signal.begin(), signal.end(), -95);
            }
            // Range Scan Sees a bargraph simillar to NRF24 grafic, using RSSI across frequencies
            else {
                if (!initRfModule("rx", bruceConfigPins.rfFreq)) displayError("Error starting module", true);
                // the band edges are drawn on the bottom row, so keep it empty here
                draw_rf_header(
                    String("RF RSSI ") + subghz_frequency_ranges[bruceConfigPins.rfScanRange] + " " +
                        String(multiplier) + "x",
                    ""
                );
                tft.fillRect(rf_plot_left(), top, rf_plot_width(), bot - top, bruceConfig.bgColor);
                tft.drawFastHLine(rf_plot_left(), bot, rf_plot_width(), bruceConfig.priColor);
                tft.setTextColor(rf_label_color(), bruceConfig.bgColor);
                char buf[8];
                float var = subghz_frequency_list[range_limits[bruceConfigPins.rfScanRange][0]];
                snprintf(buf, sizeof(buf), "%.3f", var);
                tft.drawString(buf, rf_plot_left(), bot + 2, 1);
                var = subghz_frequency_list[range_limits[bruceConfigPins.rfScanRange][1]];
                snprintf(buf, sizeof(buf), "%.3f", var);
                tft.drawRightString(buf, rf_plot_left() + rf_plot_width(), bot + 2, 1);
                int range = range_limits[bruceConfigPins.rfScanRange][1] -
                            range_limits[bruceConfigPins.rfScanRange][0] + 1;
                int space = rf_plot_width() / range;
                for (int i = 0; i < range; i++) {
                    tft.drawFastVLine(rf_plot_left() + space * i, bot - 4, 4, rf_grid_color());
                }
                std::fill(bar_size.begin(), bar_size.end(), 0);
            }
        }

        // draw dot graph for fixed frequency
        if (bruceConfigPins.rfFxdFreq) {
            int rssi = ELECHOUSE_cc1101.getRssi();
            tft.drawPixel(0, 0, 0); // To make sure CC1101 shared with TFT works properly
            int prev = signal[0];
            for (int i = 1; i < graph_size; i++) {
                if (EscPress || SelPress) break;
                const int x0 = axisX + (i - 1);
                const int x1 = axisX + i;
                const int curr = signal[i];
                // erase old segment between previous and current points
                tft.drawLine(x0, rssiY(prev), x1, rssiY(curr), bruceConfig.bgColor);
                const int next_val = (i == graph_size - 1) ? rssi : signal[i + 1];
                // shift buffer left by one
                signal[i - 1] = curr;
                if (i == graph_size - 1) signal[i] = rssi;
                // draw updated segment using new values
                tft.drawLine(x0, rssiY(curr), x1, rssiY(next_val), bruceConfig.priColor);
                prev = curr;
            }
            tft.drawFastVLine(axisX, top, bot - top, bruceConfig.priColor);
            vTaskDelay(pdMS_TO_TICKS(75));
        }
        // draw a bargraph similar to nrf24 across the range
        else {
            int range = range_limits[bruceConfigPins.rfScanRange][1] -
                        range_limits[bruceConfigPins.rfScanRange][0] + 1;

            int space = rf_plot_width() / range;
            int barW = max(1, space - 1);
            int max_idx = 0;
            for (int i = 0; i < range; i++) {
                if (EscPress || SelPress || AnyKeyPress) break;
                setMHZ(subghz_frequency_list[range_limits[bruceConfigPins.rfScanRange][0] + i]);
                delayMicroseconds(900);
                int rssi = ELECHOUSE_cc1101.getRssi();
                tft.drawPixel(0, 0, 0); // To make sure CC1101 shared with TFT works properly
                int size = map(constrain(rssi, -95, -20), -95, -20, 0, max_bar_size);
                size = constrain(size * multiplier, 0, max_bar_size);
                if (size > bar_size[i]) bar_size[i] = size;
                else bar_size[i] = bar_size[i] - (bar_size[i] - size) / 2; // slow down decrease
                bar_size[i] = constrain(bar_size[i], 0, max_bar_size);
                tft.fillRect(
                    rf_plot_left() + i * space, bot - bar_size[i], barW, bar_size[i], bruceConfig.priColor
                );
                tft.fillRect(
                    rf_plot_left() + i * space, top, space, max_bar_size - bar_size[i], bruceConfig.bgColor
                );
                if (bar_size[i] > bar_size[max_idx] && bar_size[i] > min_value) max_idx = i;
            }
            if (bar_size[max_idx] > min_value) {
                char buf[8];
                float var = subghz_frequency_list[range_limits[bruceConfigPins.rfScanRange][0] + max_idx];
                snprintf(buf, sizeof(buf), "%.2f", var);
                tft.setTextColor(rf_label_color(), bruceConfig.bgColor);
                tft.drawCentreString("Max=       ", tftWidth / 2, bot + 2, 1);
                tft.drawCentreString("Max=" + String(buf), tftWidth / 2, bot + 2, 1);
            }
        }
    }
    deinitRfModule();
#else
    displayError("Not available on Launcher version");
#endif
}
