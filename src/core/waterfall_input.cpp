#include "waterfall_input.h"

#include "core/mykeyboard.h"
#include <globals.h>

void pollWaterfallInput(WaterfallInput &in) {
    static uint32_t lastSelectMs = 0;
    bool consumed = false;
    bool sel = false;

#ifdef HAS_ENCODER
    // Rotary movement is accumulated independently of the one-shot button
    // flags; NextPress/PrevPress are duplicates of it, so drop those.
    int32_t rotarySteps = drainRotarySteps();
    if (rotarySteps < 0) in.next += -rotarySteps;
    else if (rotarySteps > 0) in.prev += rotarySteps;
    NextPress = false;
    PrevPress = false;
    bool prevNow = false;
    bool nextNow = false;
#else
    bool nextNow = NextPress;
    bool prevNow = PrevPress;
    if (nextNow) {
        in.next++;
        NextPress = false;
        consumed = true;
    }
    if (prevNow) {
        in.prev++;
        PrevPress = false;
        consumed = true;
    }
#endif
    // Cardputer ';' and '.' raise Prev+Up / Next+Down together. Count such a
    // press once, as Prev/Next, so one key does not both move and pan.
    if (UpPress) {
        if (!prevNow) in.up++;
        UpPress = false;
        consumed = true;
    }
    if (DownPress) {
        if (!nextNow) in.down++;
        DownPress = false;
        consumed = true;
    }
    if (SelPress) {
        sel = true;
        SelPress = false;
        consumed = true;
    }
    if (EscPress) {
        in.esc = true;
        EscPress = false;
        consumed = true;
    }
    if (KeyStroke.pressed) {
        keyStroke k = _getKeyPress();
        for (auto ch : k.word) {
            char lowerKey = tolower(ch);
            if (lowerKey == 'g' || lowerKey == 'a' || lowerKey == 's') in.gain++;
            else if (ch == '+' || ch == '=' || ch == ']') in.zoomIn++;
            else if (ch == '-' || ch == '_' || ch == '[') in.zoomOut++;
        }
        // Cardputer Enter is reported in the keystroke as well as SelPress.
        if (k.enter) sel = true;
        consumed = true;
    }
    // SelPress and the Enter keystroke can be seen in separate polls for the
    // same press; treat anything within the debounce window as one toggle.
    if (sel && millis() - lastSelectMs > 150) {
        in.select++;
        lastSelectMs = millis();
    }
    // Let the input task read the next key immediately instead of waiting out
    // its 75 ms AnyKeyPress hold-off.
    if (consumed) AnyKeyPress = false;
}

String waterfallControlBar(const String *items, int count, int selected, bool locked) {
    String bar;
    for (int i = 0; i < count; i++) {
        if (i > 0) bar += " ";
        if (i == selected) bar += locked ? "[" : ">";
        bar += items[i];
        if (i == selected) bar += locked ? "]" : "<";
    }
    return bar;
}
