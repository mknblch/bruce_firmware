#pragma once

#include <Arduino.h>

// Input and footer helpers shared by the SpectrumPlot waterfalls (RF, LoRa).
//
// Navigation flags are short pulses: the input task wipes them on its next
// pass (at most ~75 ms later), which is shorter than a waterfall frame. So
// they are collected into counters many times per frame — between sweep bins
// and drawing steps — and applied once at the top of the next frame, instead
// of being sampled once per frame and mostly missed.
//
// Typical use:
//     WaterfallInput pending;
//     while (...) {
//         pollWaterfallInput(pending);
//         WaterfallInput in = pending;
//         pending = WaterfallInput();
//         ... apply `in` ...
//         ... sweep / draw, calling pollWaterfallInput(pending) in between ...
//     }
struct WaterfallInput {
    int next = 0;
    int prev = 0;
    int up = 0;   // dedicated Up button only, never the Cardputer ';' alias
    int down = 0; // dedicated Down button only, never the Cardputer '.' alias
    int zoomIn = 0;
    int zoomOut = 0;
    int gain = 0; // g / a / s shortcuts
    int select = 0;
    bool esc = false;
};

void pollWaterfallInput(WaterfallInput &in);

// Footer listing the controls: ">ITEM<" marks the selection, "[ITEM]" when locked.
String waterfallControlBar(const String *items, int count, int selected, bool locked);

static inline int wrapIndex(int value, int count) { return ((value % count) + count) % count; }
