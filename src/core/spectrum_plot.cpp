#include "spectrum_plot.h"

#include "core/display.h"
#include <globals.h>

// Waterfall intensity ramp, quantised so identical columns collapse into runs.
static const int SP_HEAT_N = 16;
static uint16_t sp_heat[SP_HEAT_N];
// Envelope value (0-100) straight to its heat colour, saving a divide per pixel.
static uint16_t sp_heat_lut[101];

static void sp_build_heat_lut() {
    for (int v = 0; v <= 100; v++) sp_heat_lut[v] = sp_heat[v * (SP_HEAT_N - 1) / 100];
}

uint16_t SpectrumPlot::alertColor() {
    uint16_t pri = bruceConfig.priColor;
    int r = (pri >> 11) & 0x1f, g = (pri >> 5) & 0x3f, b = pri & 0x1f;
    // A red alert would vanish on a red theme, so fall back to amber there.
    bool reddish = (r > 18 && (g >> 1) < 12 && b < 12);
    return reddish ? TFT_ORANGE : TFT_RED;
}

void SpectrumPlot::buildGeometry() {
    _plotL = 8;
    _plotW = tftWidth - 16;
    if (_plotW < 32) {
        _plotL = 2;
        _plotW = tftWidth - 4;
    }

    int top = _hasTitle ? (BORDER_PAD_Y + 8 * FM + 2) : 3;
    _footY = tftHeight - 8 * FP - (_hasTitle ? 8 : 4);
    _lblY = _footY - 8 * FP - 2;

    int avail = _lblY - top - 2;
    if (avail < 20) { // no room for the ruler
        _lblY = -1;
        avail = _footY - top - 2;
    }
    if (avail < 14) { // no room for the status line either
        _footY = -1;
        avail = tftHeight - (_hasTitle ? 6 : 2) - top;
    }
    if (avail < 8) avail = 8;

    _wfRows = 0;
    if (avail >= 36) {
        if (_waterfallPriority) {
            int traceH = max(20, min(28, avail / 4));
            _wfRows = avail - traceH - 2;
            if (_wfRows < 0) _wfRows = 0;
        } else {
            _wfRows = avail / 3;
            if (_wfRows > 24) _wfRows = 24;
        }
    }
    _specTop = top;
    _specH = avail - _wfRows - (_wfRows ? 2 : 0);
    _specBot = _specTop + _specH - 1;
    _wfTop = _specBot + 3;
}

// Classic SDR waterfall colourmap (à la GQRX / SDR# / SDRPlay): the noise floor
// blends with the panel, then the signal climbs through blue, cyan, green,
// yellow and red to a white-hot peak. Grounded in the common SDR gradients.
static void sp_build_sdr_palette() {
    static const uint8_t stopPos[] = {0, 30, 60, 105, 150, 195, 225, 255};
    static const uint8_t stopR[] = {0, 0, 0, 0, 40, 235, 235, 255};
    static const uint8_t stopG[] = {0, 0, 40, 180, 205, 235, 70, 255};
    static const uint8_t stopB[] = {0, 90, 175, 200, 60, 0, 0, 255};
    const int NS = sizeof(stopPos) / sizeof(stopPos[0]);

    sp_heat[0] = bruceConfig.bgColor; // "no signal" blends with the panel
    for (int i = 1; i < SP_HEAT_N; i++) {
        int t = i * 255 / (SP_HEAT_N - 1);
        int s = 0;
        while (s < NS - 2 && t > stopPos[s + 1]) s++;
        int span = stopPos[s + 1] - stopPos[s];
        int f = span ? (t - stopPos[s]) * 255 / span : 0;
        int r = stopR[s] + (stopR[s + 1] - stopR[s]) * f / 255;
        int g = stopG[s] + (stopG[s + 1] - stopG[s]) * f / 255;
        int b = stopB[s] + (stopB[s + 1] - stopB[s]) * f / 255;
        sp_heat[i] = tft.color565(r, g, b);
    }
}

void SpectrumPlot::buildPalette() {
    uint16_t pri = bruceConfig.priColor;
    _bg = bruceConfig.bgColor;
    _trace = pri;
    _body = blendColors(_bg, pri, 95);
    _bodyHl = blendColors(_bg, pri, 165);
    _peak = blendColors(pri, TFT_WHITE, 150);
    _grid = blendColors(_bg, pri, 55);
    _label = blendColors(_bg, pri, 170);
    if (_sdr) sp_build_sdr_palette();
    else buildHeatPalette(sp_heat, SP_HEAT_N);
    sp_build_heat_lut();
}

void SpectrumPlot::pushPixels(int32_t x, int32_t y, int32_t w, int32_t h) {
    bool swap = tft.getSwapBytes();
    tft.setSwapBytes(true); // _line holds native RGB565 values
    tft.pushImage(x, y, w, h, _line);
    tft.setSwapBytes(swap);
}

bool SpectrumPlot::begin(const String &title, bool sdrWaterfall, bool waterfallPriority) {
    _sdr = sdrWaterfall;
    _hasTitle = (title.length() > 0);
    _waterfallPriority = waterfallPriority;
    buildGeometry();
    buildPalette();

    if (_plotW < 8) return false;

    _lineLen = max(_plotW, _specH);
    free(_line);
    _line = (uint16_t *)malloc((size_t)_lineLen * sizeof(uint16_t));
    if (!_line) return false;

    if (_wfRows) {
        _wf = (uint8_t *)calloc((size_t)_wfRows * _plotW, 1);
        if (!_wf) _wfRows = 0; // degrade to a plot without history rather than fail
    }
    _wfHead = 0;
    _ok = true;

    redraw(title);
    return true;
}

void SpectrumPlot::redraw(const String &title) {
    if (!_ok) return;
    if (_hasTitle && title.length() > 0) {
        drawMainBorderWithTitle(title);
    } else {
        tft.drawPixel(0, 0, 0);
        tft.fillScreen(_bg);
    }
    tft.fillRect(_plotL, _specTop, _plotW, tftHeight - (_hasTitle ? BORDER_PAD_Y : 2) - _specTop, _bg);
    if (_wfRows > 0) {
        tft.drawFastHLine(_plotL, _specBot + 1, _plotW, _grid);
        tft.drawFastHLine(_plotL, _specBot + 2, _plotW, _bg);
    }
    drawWaterfall();
}

void SpectrumPlot::end() {
    free(_wf);
    _wf = nullptr;
    free(_line);
    _line = nullptr;
    _lineLen = 0;
    _wfRows = 0;
    _ok = false;
}

void SpectrumPlot::trace(const uint8_t *env, const uint8_t *envPeak, int hlL, int hlR, bool alert) {
    if (!_ok || !env) return;

    uint16_t trace = alert ? alertColor() : _trace;
    uint16_t bodyHl = alert ? blendColors(_bg, trace, 165) : _bodyHl;

    int gy[3];
    gy[0] = _specBot - _specH / 4;
    gy[1] = _specBot - _specH / 2;
    gy[2] = _specBot - (_specH * 3) / 4;

    // Every pixel of the band is written exactly once per frame, which keeps the
    // animation flicker free without needing a full-screen sprite. Each column
    // is composed in _line and sent as one transfer rather than up to a dozen
    // separate pixel/line primitives.
    for (int i = 0; i < _plotW; i++) {
        int x = _plotL + i;

        int hLive = constrain((int)env[i] * (_specH - 1) / 100, 0, _specH - 1);
        int hPeak = envPeak ? constrain((int)envPeak[i] * (_specH - 1) / 100, 0, _specH - 1) : 0;
        if (hPeak < hLive) hPeak = hLive;

        // _line[0] is _specTop, _line[_specH - 1] is _specBot
        int yLive = _specH - 1 - hLive;
        int yPeak = _specH - 1 - hPeak;
        uint16_t body = (i >= hlL && i <= hlR) ? bodyHl : _body;

        for (int y = 0; y < yLive; y++) _line[y] = _bg;
        if (hPeak > hLive) _line[yPeak] = _peak;
        _line[yLive] = trace;
        for (int y = yLive + 1; y < _specH; y++) _line[y] = body;

        // dashed reference grid, visible only through the empty sky
        if ((i & 3) == 0) {
            for (int k = 0; k < 3; k++) {
                int gyk = gy[k] - _specTop;
                if (gyk > 0 && gyk < yLive - 1 && gyk != yPeak) _line[gyk] = _grid;
            }
        }

        pushPixels(x, _specTop, 1, _specH);
    }
}

// Newest row sits right under the trace baseline and older ones fall away.
void SpectrumPlot::drawWaterfall() {
    if (!_wfRows || !_wf) return;

    tft.drawFastHLine(_plotL, _specBot + 1, _plotW, _grid);
    tft.drawFastHLine(_plotL, _specBot + 2, _plotW, _bg);

    // One transfer per row: the rows are re-rendered every frame as the
    // history scrolls, so per-run line primitives cost thousands of SPI
    // transactions on noisy data.
    for (int r = 0; r < _wfRows; r++) {
        int idx = (_wfHead - r + 2 * _wfRows) % _wfRows;
        const uint8_t *row = _wf + (size_t)idx * _plotW;
        for (int i = 0; i < _plotW; i++) _line[i] = sp_heat_lut[row[i] > 100 ? 100 : row[i]];
        pushPixels(_plotL, _wfTop + r, _plotW, 1);
    }
}

void SpectrumPlot::pushRow(const uint8_t *env) {
    if (!_ok || !_wfRows || !_wf || !env) return;
    _wfHead = (_wfHead + 1) % _wfRows;
    memcpy(_wf + (size_t)_wfHead * _plotW, env, _plotW);
    drawWaterfall();
}

void SpectrumPlot::ruler(const int *cols, const String *labels, int count, int highlight) {
    if (!_ok || _lblY < 0 || !cols || !labels) return;

    int topY = _wfRows ? (_wfTop + _wfRows) : (_specBot + 1);
    int botY = (_footY >= 0) ? (_footY - 1) : (tftHeight - (_hasTitle ? BORDER_PAD_Y : 2));
    int h = botY - topY + 1;
    if (h > 0) {
        tft.fillRect(_plotL, topY, _plotW, h, _bg);
    }
    tft.setTextSize(FP);

    for (int i = 0; i < count; i++) {
        int w = labels[i].length() * FP * LW;
        int tx = _plotL + cols[i] - w / 2;
        // keep edge labels inside the plot
        if (tx < _plotL) tx = _plotL;
        if (tx + w > _plotL + _plotW) tx = _plotL + _plotW - w;

        if (i == highlight) {
            tft.fillRect(tx - 2, _lblY - 1, w + 4, 8 * FP + 2, bruceConfig.priColor);
            tft.setTextColor(_bg, bruceConfig.priColor);
        } else {
            tft.setTextColor(_label, _bg);
        }
        tft.drawString(labels[i], tx, _lblY, 1);
    }
}

void SpectrumPlot::status(const String &text, bool alert) {
    if (!_ok || _footY < 0) return;

    int botY = tftHeight - (_hasTitle ? BORDER_PAD_Y : 2);
    int h = botY - _footY + 1;
    tft.fillRect(_plotL, _footY, _plotW, max(h, 8 * FP), _bg);
    tft.setTextSize(FP);
    tft.setTextColor(alert ? alertColor() : _label, _bg);

    String s = text;
    int maxChars = _plotW / (FP * LW);
    if ((int)s.length() > maxChars) s = s.substring(0, maxChars);
    tft.drawString(s, _plotL, _footY, 1);
}
