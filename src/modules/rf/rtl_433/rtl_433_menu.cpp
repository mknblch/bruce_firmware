// SPDX-License-Identifier: AGPL-3.0-or-later
#include "rtl_433_menu.h"
#include "core/display.h"
#include "core/led_control.h"
#include "core/mykeyboard.h"
#include "core/sd_functions.h"
#include "core/settings.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <algorithm>

static void rf_clear_nav_state() {
    NextPress = false;
    PrevPress = false;
    UpPress = false;
    DownPress = false;
    SelPress = false;
    EscPress = false;
    AnyKeyPress = false;
}

static void rf_wait_any_key() {
    uint32_t quietSince = millis();
    uint32_t deadline = quietSince + 30000;
    while (millis() - quietSince < 250 && millis() < deadline) {
        if (AnyKeyPress || SelPress || EscPress || NextPress || PrevPress || UpPress || DownPress) {
            rf_clear_nav_state();
            quietSince = millis();
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    while (millis() < deadline) {
        if (AnyKeyPress || SelPress || EscPress || NextPress || PrevPress || UpPress || DownPress) {
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void show_reading_details(int index) {
    Rtl433Engine &engine = Rtl433Engine::instance();
    const Rtl433Reading *ptr = engine.getRecentAt(index);
    if (!ptr) return;
    Rtl433Reading r = *ptr;

    rf_clear_nav_state();
    bool exitView = false;

    while (!exitView) {
        drawMainBorderWithTitle(r.protocol);

        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
        tft.setTextSize(FP);

        setPadCursor(1, 0);
        padprintln("Model: " + r.model);
        padprintln("ID: " + String(r.device_id) + (r.channel >= 0 ? (" Ch:" + String(r.channel)) : ""));

        if (r.has_temp) {
            padprintln("Temp: " + String(r.temp_c, 1) + " C (" + String(r.temp_f, 1) + " F)");
        }
        if (r.has_humidity) {
            padprintln("Humidity: " + String((int)r.humidity) + " %");
        }
        if (r.has_pressure) {
            padprintln("Press: " + String(r.pressure_psi, 1) + " psi (" + String(r.pressure_kpa, 0) + " kPa)");
        }
        if (r.has_battery) {
            padprintln("Battery: " + String(r.battery_ok ? "OK" : "LOW"));
        }
        if (r.has_wind) {
            padprintln("Wind: " + String(r.wind_speed_ms, 1) + " m/s (G:" + String(r.wind_gust_ms, 1) + ")");
        }
        if (r.has_status && r.status_str.length() > 0) {
            padprintln("Status: " + r.status_str);
        }
        padprintln("Freq: " + String(r.frequency, 2) + " MHz (" + r.modulation + ")");
        padprintln("Payload: " + r.payload_hex);

        tft.setTextColor(getColorVariation(bruceConfig.priColor), bruceConfig.bgColor);
        tft.drawCentreString(
            "[OK] Menu   [< / ESC] Back", tftWidth / 2, tftHeight - BORDER_PAD_X - FP * LH, SMOOTH_FONT
        );

        rf_clear_nav_state();
        delay(100);

        while (1) {
            if (check(EscPress) || check(PrevPress) || check(UpPress) || check(PrevPagePress)) {
                exitView = true;
                break;
            }
            if (check(SelPress)) {
                enum Action { ACT_NONE, ACT_REPLAY, ACT_SAVE_ONE, ACT_SAVE_ALL, ACT_DUMP, ACT_CLEAR, ACT_BACK };
                Action chosenAction = ACT_NONE;

                std::vector<Option> opts = {
                    {"Replay RF",        [&]() { chosenAction = ACT_REPLAY; }},
                    {"Save Packet",      [&]() { chosenAction = ACT_SAVE_ONE; }},
                    {"Save All",         [&]() { chosenAction = ACT_SAVE_ALL; }},
                    {"Dump JSON",        [&]() { chosenAction = ACT_DUMP; }},
                    {"Clear Results",    [&]() { chosenAction = ACT_CLEAR; }},
                    {"Go Back",          [&]() { chosenAction = ACT_BACK; }},
                };

                loopOptions(opts, MENU_TYPE_SUBMENU, r.protocol.c_str());

                switch (chosenAction) {
                    case ACT_REPLAY:
                        displayTextLine("Replaying RF...");
                        engine.replayReading(r);
                        delay(600);
                        break;
                    case ACT_SAVE_ONE: {
                        String savedPath;
                        if (engine.saveSubFile(r, &savedPath)) {
                            displaySuccess("Saved: " + savedPath, true);
                        } else {
                            displayError("Save failed", true);
                        }
                        break;
                    }
                    case ACT_SAVE_ALL: {
                        int saved = 0;
                        engine.saveAllSubFiles(&saved);
                        if (saved > 0) {
                            displaySuccess("Saved " + String(saved) + " in BruceRF", true);
                        } else {
                            displayError("Save failed", true);
                        }
                        break;
                    }
                    case ACT_DUMP:
                        Serial.println(r.toJson());
                        displayInfo("Dumped to Serial", true);
                        break;
                    case ACT_CLEAR:
                        engine.clearRecent();
                        displaySuccess("Cleared List", true);
                        exitView = true;
                        break;
                    case ACT_BACK:
                    case ACT_NONE:
                    default:
                        if (check(EscPress) || returnToMenu) {
                            exitView = true;
                        }
                        break;
                }
                rf_clear_nav_state();
                break;
            }
            delay(10);
        }
    }
    rf_clear_nav_state();
}

static void view_recent_packets_menu() {
    Rtl433Engine &engine = Rtl433Engine::instance();
    bool exitRecent = false;
    while (!exitRecent) {
        size_t count = engine.getRecentCount();
        if (count == 0) {
            displayInfo("No packets captured yet", true);
            return;
        }

        std::vector<Option> opts;
        for (size_t i = 0; i < count; i++) {
            size_t idx = count - 1 - i; // newest first
            const Rtl433Reading *r = engine.getRecentAt(idx);
            if (r) {
                String line = r->toSummaryLine();
                opts.push_back({line, [idx]() { show_reading_details(idx); }});
            }
        }
        opts.push_back({"Save All (.SUB)", [&]() {
            int saved = 0;
            engine.saveAllSubFiles(&saved);
            if (saved > 0) {
                displaySuccess("Saved " + String(saved) + " in BruceRF", true);
            } else {
                displayError("Save failed", true);
            }
        }});
        opts.push_back({"Clear Results", [&]() {
            Rtl433Engine::instance().clearRecent();
            displaySuccess("Cleared List", true);
        }});
        opts.push_back({"Go Back", [&]() {
            exitRecent = true;
        }});

        int res = loopOptions(opts, MENU_TYPE_SUBMENU, "Recent RTL433");
        if (check(EscPress) || res < 0 || returnToMenu || exitRecent || engine.getRecentCount() == 0) {
            returnToMenu = false;
            break;
        }
    }
}

static std::vector<float> get_band_frequencies(Rtl433Band band) {
    static const float minFrequency[] = {300.0f, 387.0f, 779.0f};
    static const float maxFrequency[] = {348.0f, 464.0f, 928.0f};
    int bandIndex = (int)band;
    if (bandIndex < 0 || bandIndex >= RTL433_BAND_COUNT) return {};

    std::vector<float> frequencies;
    for (int i = range_limits[bandIndex][0]; i <= range_limits[bandIndex][1]; i++) {
        float frequency = subghz_frequency_list[i];
        if (frequency >= minFrequency[bandIndex] && frequency <= maxFrequency[bandIndex]) {
            frequencies.push_back(frequency);
        }
    }
    return frequencies;
}

static bool same_preset_mode(const Rtl433PresetDef *left, const Rtl433PresetDef *right) {
    return left->modulation == right->modulation && left->deviation == right->deviation &&
           left->rx_bw == right->rx_bw && left->data_rate == right->data_rate;
}

static std::vector<int> unique_preset_modes(const std::vector<int> &presets) {
    std::vector<int> modes;
    for (int preset : presets) {
        const Rtl433PresetDef *candidate = rtl433_get_preset_def(preset);
        bool duplicate = false;
        for (int selected : modes) {
            if (same_preset_mode(candidate, rtl433_get_preset_def(selected))) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) modes.push_back(preset);
    }
    return modes;
}

static bool is_band_range_profile(int profile) {
    switch (profile) {
        case RTL433_HOP_300_BAND_OOK:
        case RTL433_HOP_300_BAND_FSK:
        case RTL433_HOP_300_BAND_GFSK:
        case RTL433_HOP_300_BAND_ALL:
        case RTL433_HOP_400_BAND_OOK:
        case RTL433_HOP_400_BAND_FSK:
        case RTL433_HOP_400_BAND_GFSK:
        case RTL433_HOP_400_BAND_MSK:
        case RTL433_HOP_400_BAND_ALL:
        case RTL433_HOP_800_BAND_OOK:
        case RTL433_HOP_800_BAND_FSK:
        case RTL433_HOP_800_BAND_GFSK:
        case RTL433_HOP_800_BAND_MSK:
        case RTL433_HOP_800_BAND_ALL:
            return true;
        default:
            return false;
    }
}

static void select_freq_and_mode_menu(Rtl433Band band) {
    Rtl433Engine &engine = Rtl433Engine::instance();
    std::vector<Option> opts;
    std::vector<float> frequencies = get_band_frequencies(band);

    float selectedFrequency = 0.0f;
    for (float frequency : frequencies) {
        String label = String(frequency, 3) + " MHz";
        opts.push_back({label, [frequency, &selectedFrequency]() { selectedFrequency = frequency; }});
    }
    opts.push_back({"Go Back", []() {}});
    loopOptions(opts, MENU_TYPE_SUBMENU, "1 Freq + 1 Mode");

    if (selectedFrequency <= 0.0f) return;

    std::vector<int> modePresets;
    for (int preset : rtl433_get_band_presets(band)) {
        const Rtl433PresetDef *candidate = rtl433_get_preset_def(preset);
        bool duplicate = false;
        for (int selected : modePresets) {
            const Rtl433PresetDef *existing = rtl433_get_preset_def(selected);
            if (candidate->modulation == existing->modulation && candidate->deviation == existing->deviation &&
                candidate->rx_bw == existing->rx_bw && candidate->data_rate == existing->data_rate) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) modePresets.push_back(preset);
    }

    opts.clear();
    for (int preset : modePresets) {
        const Rtl433PresetDef *pdef = rtl433_get_preset_def(preset);
        String modeName;
        switch (pdef->modulation) {
            case 0: modeName = "2-FSK"; break;
            case 1: modeName = "GFSK"; break;
            case 2: modeName = "OOK"; break;
            case 4: modeName = "MSK"; break;
            default: modeName = "Mode"; break;
        }
        if (pdef->modulation != 2) modeName += " " + String(pdef->data_rate, 3) + " kbps";
        opts.push_back({modeName, [preset, selectedFrequency, modeName, &engine]() {
            engine.isChangingPreset = false;
            engine.currentPreset = preset;
            engine.currentFrequency = selectedFrequency;
            displaySuccess("Fixed: " + String(selectedFrequency, 3) + " MHz (" + modeName + ")", true);
        }});
    }
    opts.push_back({"Go Back", []() {}});
    loopOptions(opts, MENU_TYPE_SUBMENU, (String(selectedFrequency, 3) + " MHz - Select Mode").c_str());
}

static void select_freq_all_modes_menu(Rtl433Band band) {
    Rtl433Engine &engine = Rtl433Engine::instance();
    std::vector<Option> opts;
    std::vector<float> frequencies = get_band_frequencies(band);

    for (float frequency : frequencies) {
        String label = String(frequency, 3) + " MHz (All Modes)";
        opts.push_back({label, [&engine, band, frequency]() {
            engine.isChangingPreset = true;
            engine.changingPreset = RTL433_HOP_SINGLE_FREQ_ALL;
            engine.hopGroup = RTL433_HOP_SINGLE_FREQ_ALL;
            engine.hopFrequencyBand = band;
            engine.hopFrequency = frequency;
            displaySuccess(String(frequency, 3) + " MHz All Modes", true);
        }});
    }

    opts.push_back({"Go Back", []() {}});
    loopOptions(opts, MENU_TYPE_SUBMENU, "1 Freq + All Modes");
}

static void select_range_single_mode_menu(Rtl433Band band) {
    Rtl433Engine &engine = Rtl433Engine::instance();
    std::vector<Option> opts;

    if (band == RTL433_BAND_300) {
        opts.push_back({"300 Band (OOK)", [&engine]() {
            engine.isChangingPreset = true;
            engine.changingPreset = RTL433_HOP_300_BAND_OOK;
            engine.hopGroup = RTL433_HOP_300_BAND_OOK;
            displaySuccess("300 Band (OOK)", true);
        }});
        opts.push_back({"300 Band (2-FSK)", [&engine]() {
            engine.isChangingPreset = true;
            engine.changingPreset = RTL433_HOP_300_BAND_FSK;
            engine.hopGroup = RTL433_HOP_300_BAND_FSK;
            displaySuccess("300 Band (2-FSK)", true);
        }});
        opts.push_back({"300 Band (GFSK)", [&engine]() {
            engine.isChangingPreset = true;
            engine.changingPreset = RTL433_HOP_300_BAND_GFSK;
            engine.hopGroup = RTL433_HOP_300_BAND_GFSK;
            displaySuccess("300 Band (GFSK)", true);
        }});
    } else if (band == RTL433_BAND_400) {
        opts.push_back({"400 Band (OOK)", [&engine]() {
            engine.isChangingPreset = true;
            engine.changingPreset = RTL433_HOP_400_BAND_OOK;
            engine.hopGroup = RTL433_HOP_400_BAND_OOK;
            displaySuccess("400 Band (OOK)", true);
        }});
        opts.push_back({"400 Band (2-FSK)", [&engine]() {
            engine.isChangingPreset = true;
            engine.changingPreset = RTL433_HOP_400_BAND_FSK;
            engine.hopGroup = RTL433_HOP_400_BAND_FSK;
            displaySuccess("400 Band (2-FSK)", true);
        }});
        opts.push_back({"400 Band (GFSK)", [&engine]() {
            engine.isChangingPreset = true;
            engine.changingPreset = RTL433_HOP_400_BAND_GFSK;
            engine.hopGroup = RTL433_HOP_400_BAND_GFSK;
            displaySuccess("400 Band (GFSK)", true);
        }});
        opts.push_back({"400 Band (MSK)", [&engine]() {
            engine.isChangingPreset = true;
            engine.changingPreset = RTL433_HOP_400_BAND_MSK;
            engine.hopGroup = RTL433_HOP_400_BAND_MSK;
            displaySuccess("400 Band (MSK)", true);
        }});
    } else if (band == RTL433_BAND_800) {
        opts.push_back({"800 Band (OOK)", [&engine]() {
            engine.isChangingPreset = true;
            engine.changingPreset = RTL433_HOP_800_BAND_OOK;
            engine.hopGroup = RTL433_HOP_800_BAND_OOK;
            displaySuccess("800 Band (OOK)", true);
        }});
        opts.push_back({"800 Band (2-FSK)", [&engine]() {
            engine.isChangingPreset = true;
            engine.changingPreset = RTL433_HOP_800_BAND_FSK;
            engine.hopGroup = RTL433_HOP_800_BAND_FSK;
            displaySuccess("800 Band (2-FSK)", true);
        }});
        opts.push_back({"800 Band (GFSK)", [&engine]() {
            engine.isChangingPreset = true;
            engine.changingPreset = RTL433_HOP_800_BAND_GFSK;
            engine.hopGroup = RTL433_HOP_800_BAND_GFSK;
            displaySuccess("800 Band (GFSK)", true);
        }});
        opts.push_back({"800 Band (MSK)", [&engine]() {
            engine.isChangingPreset = true;
            engine.changingPreset = RTL433_HOP_800_BAND_MSK;
            engine.hopGroup = RTL433_HOP_800_BAND_MSK;
            displaySuccess("800 Band (MSK)", true);
        }});
    }

    opts.push_back({"Go Back", []() {}});
    loopOptions(opts, MENU_TYPE_SUBMENU, "Range + 1 Mode");
}

static void select_band_menu(Rtl433Band band) {
    Rtl433Engine &engine = Rtl433Engine::instance();
    bool exitBand = false;
    const char *bandTitle = rtl433_get_band_name(band);

    while (!exitBand) {
        int allHopPreset = (band == RTL433_BAND_300) ? RTL433_HOP_300_BAND_ALL :
                           (band == RTL433_BAND_400) ? RTL433_HOP_400_BAND_ALL : RTL433_HOP_800_BAND_ALL;

        std::vector<Option> opts = {
            {"1 Freq + 1 Mode",   [band]() { select_freq_and_mode_menu(band); }},
            {"1 Freq + All Modes",[band]() { select_freq_all_modes_menu(band); }},
            {"Range + 1 Mode",    [band]() { select_range_single_mode_menu(band); }},
            {"Range + All Modes", [allHopPreset, &engine, bandTitle]() {
                engine.isChangingPreset = true;
                engine.changingPreset = allHopPreset;
                engine.hopGroup = allHopPreset;
                displaySuccess(String(bandTitle) + " All", true);
            }},
            {"Go Back",           [&]() { exitBand = true; }},
        };

        int res = loopOptions(opts, MENU_TYPE_SUBMENU, bandTitle);
        if (check(EscPress) || res < 0 || returnToMenu || exitBand) {
            returnToMenu = false;
            break;
        }
    }
}

static void select_hop_timeout_menu() {
    Rtl433Engine &engine = Rtl433Engine::instance();
    struct TimeoutOpt {
        uint32_t ms;
        const char *name;
    };
    static const TimeoutOpt timeoutOpts[] = {
        {3000,   "3 seconds (Fast)"},
        {5000,   "5 seconds"},
        {10000,  "10 seconds (Standard)"},
        {15000,  "15 seconds"},
        {30000,  "30 seconds (Sensor Cycle)"},
        {60000,  "60 seconds (1 minute)"},
        {120000, "120 seconds (2 minutes)"},
    };
    std::vector<Option> opts;
    for (size_t i = 0; i < sizeof(timeoutOpts) / sizeof(timeoutOpts[0]); i++) {
        uint32_t ms = timeoutOpts[i].ms;
        const char *label = timeoutOpts[i].name;
        opts.push_back({label, [&engine, ms, label]() {
            engine.hopTimeoutMs = ms;
            displaySuccess(String("Interval: ") + label, true);
        }});
    }
    opts.push_back({"Go Back", []() {}});
    loopOptions(opts, MENU_TYPE_SUBMENU, "Hop Interval");
}

void rtl433_presets_menu() {
    Rtl433Engine &engine = Rtl433Engine::instance();
    bool exitPresets = false;

    while (!exitPresets) {
        String modeStatus = engine.isChangingPreset ?
            (String("Hop: ") + rtl433_get_changing_preset_name(engine.changingPreset)) :
            (String("Fixed: ") + rtl433_get_preset_name(engine.currentPreset));
        String intervalStr = "Hop Interval (" + String(engine.hopTimeoutMs / 1000) + "s)";

        std::vector<Option> opts = {
            {"300 Band (300-348M)",                                                  []() { select_band_menu(RTL433_BAND_300); } },
            {"400 Band (387-464M)",                                                  []() { select_band_menu(RTL433_BAND_400); } },
            {"800 Band (779-928M)",                                                  []() { select_band_menu(RTL433_BAND_800); } },
            {"All Bands (All Modes)",                                                [&]() {
                engine.isChangingPreset = true;
                engine.changingPreset = RTL433_HOP_ALL_BANDS_ALL;
                engine.hopGroup = RTL433_HOP_ALL_BANDS_ALL;
                displaySuccess("Selected: All Bands", true);
            }},
            {intervalStr,                                                               select_hop_timeout_menu                     },
            {String("Extend on Signal: ") + (engine.hopStayOnSignal ? "[ON]" : "[OFF]"), [&]() {
                engine.hopStayOnSignal = !engine.hopStayOnSignal;
                displayInfo(String("Extend on Signal: ") + (engine.hopStayOnSignal ? "ON" : "OFF"), true);
            }},
            {"Go Back",                                                                 [&]() { exitPresets = true; }               },
        };

        int res = loopOptions(opts, MENU_TYPE_SUBMENU, "RTL433 Profiles");
        if (check(EscPress) || res < 0 || returnToMenu || exitPresets) {
            returnToMenu = false;
            break;
        }
    }
}

static void view_sd_log_menu() {
    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) {
        displayError("Storage not available", true);
        return;
    }

    String path = "/rtl433/traffic.json";
    if (!fs->exists(path)) {
        path = "/rtl433_traffic.json";
    }

    if (!fs->exists(path)) {
        displayInfo("No log file found on SD", true);
        return;
    }

    viewFile(*fs, path);
}

static void show_decoders_info() {
    rf_clear_nav_state();
    drawMainBorderWithTitle("RTL433 Decoders");
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.setTextSize(FP);
    setPadCursor(1, 0);

    padprintln("Supported Protocols:");
    padprintln("1. Nexus/Rubicson/TFA (OOK PPM)");
    padprintln("2. Acurite 606TX (OOK PWM)");
    padprintln("3. Acurite 592TXR Tower (OOK)");
    padprintln("4. Oregon v2.1/v3 (OOK Manch)");
    padprintln("5. FineOffset WH65/WH24 (FSK)");
    padprintln("6. Bresser 5/6/7in1 (GFSK/FSK)");
    padprintln("7. Wireless M-Bus (MSK Mode T/S)");
    padprintln("8. LaCrosse TX29/35 (FSK/OOK)");
    padprintln("9. Honeywell 5800 (OOK Manch)");
    padprintln("10. Schrader TPMS (OOK Manch)");
    padprintln("11. Toyota TPMS (FSK Manch)");
    padprintln("12. Kerui / EV1527 (OOK PWM)");
    padprintln("13. DSC Security (OOK Manch)");
    padprintln("14. Proove / Nexa (OOK PWM)");

    tft.setTextColor(getColorVariation(bruceConfig.priColor), bruceConfig.bgColor);
    tft.drawCentreString(
        "Press any key", tftWidth / 2, tftHeight - BORDER_PAD_X - FP * LH, SMOOTH_FONT
    );
    rf_wait_any_key();
    rf_clear_nav_state();
}

void rtl433_replay_menu() {
    Rtl433Engine &engine = Rtl433Engine::instance();
    bool exitReplay = false;

    while (!exitReplay) {
        String spreadLabel = "Freq Spread: ";
        if (engine.replayFreqSpread == 1) spreadLabel += "[+/-15 kHz]";
        else if (engine.replayFreqSpread == 2) spreadLabel += "[+/-30 kHz]";
        else if (engine.replayFreqSpread == 3) spreadLabel += "[+/-50 kHz]";
        else spreadLabel += "[OFF]";

        std::vector<Option> opts = {
            {String("Preamble Synth: ") + (engine.replayPreamble ? "[ON]" : "[OFF]"), [&]() {
                engine.replayPreamble = !engine.replayPreamble;
                displayInfo(String("Preamble: ") + (engine.replayPreamble ? "ON" : "OFF"), true);
            }},
            {spreadLabel, [&]() {
                engine.replayFreqSpread = (engine.replayFreqSpread + 1) % 4;
                String modeName = (engine.replayFreqSpread == 1) ? "+/-15 kHz (3x)" :
                                  (engine.replayFreqSpread == 2) ? "+/-30 kHz (5x)" :
                                  (engine.replayFreqSpread == 3) ? "+/-50 kHz (3x)" : "OFF (Single)";
                displayInfo("Spread: " + modeName, true);
            }},
            {String("Repeats: [") + String(engine.replayRepeats) + "x]", [&]() {
                if (engine.replayRepeats == 1) engine.replayRepeats = 3;
                else if (engine.replayRepeats == 3) engine.replayRepeats = 5;
                else if (engine.replayRepeats == 5) engine.replayRepeats = 8;
                else if (engine.replayRepeats == 8) engine.replayRepeats = 10;
                else engine.replayRepeats = 1;
                displayInfo("Repeats: " + String(engine.replayRepeats), true);
            }},
            {String("Frame Gap: [") + String(engine.replayGapMs) + "ms]", [&]() {
                if (engine.replayGapMs == 10) engine.replayGapMs = 20;
                else if (engine.replayGapMs == 20) engine.replayGapMs = 40;
                else if (engine.replayGapMs == 40) engine.replayGapMs = 60;
                else engine.replayGapMs = 10;
                displayInfo("Gap: " + String(engine.replayGapMs) + "ms", true);
            }},
            {"Go Back", [&]() { exitReplay = true; }},
        };

        int res = loopOptions(opts, MENU_TYPE_SUBMENU, "Replay Settings");
        if (check(EscPress) || res < 0 || returnToMenu || exitReplay) {
            returnToMenu = false;
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// Test Transmit menu
// ---------------------------------------------------------------------------
struct Rtl433TestTxSample {
    const char *key;        // sampleType string accepted by Rtl433Engine::transmitSample()
    const char *label;      // Short display name
    const char *modulation; // Modulation family (for display only)
};

// One representative synthetic test packet per modulation family that already has a
// working decoder in this codebase (see Rtl433Engine::transmitSample() / decode()).
static const Rtl433TestTxSample rtl433_test_tx_samples[] = {
    {"nexus",     "Nexus/Rubicson TH", "OOK"  },
    {"acurite",   "Acurite 606TX",     "OOK"  },
    {"honeywell", "Honeywell 5800",    "OOK"  },
    {"wh65",      "FineOffset WH65",   "2-FSK"},
    {"bresser",   "Bresser 5-in-1",    "GFSK" },
    {"wmbus",     "Wireless M-Bus T",  "MSK"  },
};
static const int RTL433_TEST_TX_SAMPLE_COUNT = sizeof(rtl433_test_tx_samples) / sizeof(rtl433_test_tx_samples[0]);

static void select_test_tx_sample_menu() {
    Rtl433Engine &engine = Rtl433Engine::instance();
    std::vector<Option> opts;

    for (int i = 0; i < RTL433_TEST_TX_SAMPLE_COUNT; i++) {
        const Rtl433TestTxSample &s = rtl433_test_tx_samples[i];
        String label = String(s.label) + " (" + s.modulation + ")";
        opts.push_back({label, [i, &engine]() {
            engine.testTxSampleIdx = i;
        }});
    }
    opts.push_back({"Go Back", []() {}});
    loopOptions(opts, MENU_TYPE_SUBMENU, "Select Sample");
}

static void select_test_tx_frequency_menu() {
    Rtl433Engine &engine = Rtl433Engine::instance();
    std::vector<Option> opts = {
        {"433.92 MHz (400 Band)", [&]() { engine.testTxFrequency = 433.92f; }},
        {"434.42 MHz (400 Band)", [&]() { engine.testTxFrequency = 434.42f; }},
        {"315.00 MHz (300 Band)", [&]() { engine.testTxFrequency = 315.00f; }},
        {"345.00 MHz (300 Band)", [&]() { engine.testTxFrequency = 345.00f; }},
        {"868.35 MHz (800 Band)", [&]() { engine.testTxFrequency = 868.35f; }},
        {"868.95 MHz (wM-Bus T)", [&]() { engine.testTxFrequency = 868.95f; }},
        {"868.30 MHz (wM-Bus S)", [&]() { engine.testTxFrequency = 868.30f; }},
        {"915.00 MHz (800 Band)", [&]() { engine.testTxFrequency = 915.00f; }},
    };
    opts.push_back({"Go Back", []() {}});
    loopOptions(opts, MENU_TYPE_SUBMENU, "Select Frequency");
}

void rtl433_test_tx_menu() {
    Rtl433Engine &engine = Rtl433Engine::instance();
    bool exitTestTx = false;

    while (!exitTestTx) {
        if (engine.testTxSampleIdx < 0 || engine.testTxSampleIdx >= RTL433_TEST_TX_SAMPLE_COUNT) {
            engine.testTxSampleIdx = 0;
        }
        const Rtl433TestTxSample &s = rtl433_test_tx_samples[engine.testTxSampleIdx];
        String sampleLabel = String("Sample: ") + s.label + " (" + s.modulation + ")";
        String freqLabel = "Frequency: " + String(engine.testTxFrequency, 2) + " MHz";

        std::vector<Option> opts = {
            {sampleLabel, select_test_tx_sample_menu  },
            {freqLabel,   select_test_tx_frequency_menu},
            {"Send Test Packet", [&]() {
                displayTextLine("Transmitting...");
                bool ok = engine.transmitSample(s.key, engine.testTxFrequency);
                if (ok) {
                    displaySuccess("Sent: " + String(s.label), true);
                } else {
                    displayError("Transmit failed", true);
                }
            }},
            {"Go Back",   [&]() { exitTestTx = true; }},
        };

        int res = loopOptions(opts, MENU_TYPE_SUBMENU, "RTL433 Test TX");
        if (check(EscPress) || res < 0 || returnToMenu || exitTestTx) {
            returnToMenu = false;
            break;
        }
    }
}

void rtl433_sniff_screen(bool hopping) {
    Rtl433Engine &engine = Rtl433Engine::instance();
    bool isHopping = hopping || engine.isChangingPreset;

    struct HopStep {
        float frequency;
        int preset;
    };
    std::vector<HopStep> hopList;
    size_t currentHopIdx = 0;
    int currentPreset = engine.currentPreset;
    float currentFreq = engine.currentFrequency;

    if (isHopping) {
        std::vector<int> profilePresets = rtl433_get_changing_presets(engine.changingPreset);
        if (engine.changingPreset == RTL433_HOP_SINGLE_FREQ_ALL) {
            profilePresets = unique_preset_modes(rtl433_get_band_presets(engine.hopFrequencyBand));
            for (int preset : profilePresets) {
                hopList.push_back({engine.hopFrequency, preset});
            }
        } else if (is_band_range_profile(engine.changingPreset) ||
                   engine.changingPreset == RTL433_HOP_ALL_BANDS_ALL) {
            if (engine.changingPreset == RTL433_HOP_ALL_BANDS_ALL) {
                for (Rtl433Band band : {RTL433_BAND_300, RTL433_BAND_400, RTL433_BAND_800}) {
                    std::vector<float> frequencies = get_band_frequencies(band);
                    std::vector<int> bandModes;
                    for (int preset : profilePresets) {
                        if (rtl433_get_preset_def(preset)->band == band) bandModes.push_back(preset);
                    }
                    bandModes = unique_preset_modes(bandModes);
                    for (float frequency : frequencies) {
                        for (int preset : bandModes) hopList.push_back({frequency, preset});
                    }
                }
            } else {
                profilePresets = unique_preset_modes(profilePresets);
            }
            if (engine.changingPreset != RTL433_HOP_ALL_BANDS_ALL && !profilePresets.empty()) {
                Rtl433Band band = rtl433_get_preset_def(profilePresets[0])->band;
                std::vector<float> frequencies = get_band_frequencies(band);
                for (float frequency : frequencies) {
                    for (int preset : profilePresets) hopList.push_back({frequency, preset});
                }
            }
        } else {
            for (int preset : profilePresets) {
                hopList.push_back({rtl433_get_preset_def(preset)->default_freq, preset});
            }
        }
        if (hopList.empty()) hopList.push_back({433.92f, RTL433_PRESET_OOK_433});
        currentPreset = hopList[0].preset;
        currentFreq = hopList[0].frequency;
    }

    RfRxSession rx;
    if (!rx.begin()) {
        displayError("RX Session Failed", true);
        return;
    }

    if (!engine.initRadio(currentFreq, currentPreset)) {
        rx.end();
        displayError("Radio Init Failed", true);
        return;
    }
    rx.flush();

    rf_clear_nav_state();
    bool dirty = true;
    int selectedIndex = 0; // 0 is newest packet, count-1 is oldest
    int scrollOffset = 0;  // top visible item index
    uint32_t lastRssiCheck = 0;
    int currentRssi = -90;
    uint32_t hopStart = millis();
    uint32_t lastTimerUpdate = 0;

    while (1) {
        if (check(EscPress)) break;

        uint32_t now = millis();

        // Hopping timer
        if (isHopping) {
            uint32_t elapsed = now - hopStart;
            if (elapsed >= engine.hopTimeoutMs) {
                currentHopIdx = (currentHopIdx + 1) % hopList.size();
                currentPreset = hopList[currentHopIdx].preset;
                currentFreq = hopList[currentHopIdx].frequency;
                engine.switchPreset(currentFreq, currentPreset);
                rx.flush();
                hopStart = millis();
                dirty = true;
            } else if (now - lastTimerUpdate >= 1000) {
                lastTimerUpdate = now;
                uint32_t remSec = (elapsed >= engine.hopTimeoutMs) ? 0 : ((engine.hopTimeoutMs - elapsed + 999) / 1000);
                String headerTitle = "SCAN [" + String(remSec) + "s] " + String(currentFreq, 2) + "M " +
                                     String(rtl433_get_preset_name(currentPreset));
                printTitle(headerTitle);
                tft.drawPixel(0, 0, 0);
            }
        }

        if (now - lastRssiCheck > 500) {
            lastRssiCheck = now;
            if (bruceConfigPins.rfModule == CC1101_SPI_MODULE) {
                currentRssi = ELECHOUSE_cc1101.getRssi();
            }
        }

        // 1. Hardware FIFO packet check (CC1101 FSK / GFSK / MSK)
        Rtl433Reading fifoReading;
        if (engine.pollFifo(currentFreq, currentPreset, currentRssi, fifoReading)) {
            blinkLed();
            engine.addRecent(fifoReading);
            engine.logJson(fifoReading, engine.sdLoggingEnabled);
            if (isHopping && engine.hopStayOnSignal) {
                hopStart = millis();
            }
            dirty = true;
        }

        // 2. Software pulse demodulation (OOK and fallback)
        std::vector<int> durations;
        if (rx.poll(durations)) {
            Rtl433Reading reading;
            if (engine.decode(durations, currentFreq, currentPreset, currentRssi, reading)) {
                blinkLed();
                engine.addRecent(reading);
                engine.logJson(reading, engine.sdLoggingEnabled);
                if (isHopping && engine.hopStayOnSignal) {
                    hopStart = millis(); // Extend stay on active channel
                }
                dirty = true;
            }
        }

        // Hotkey 'c' / 'C' to clear result lists
        char pressedKey = checkLetterShortcutPress();
        if (pressedKey == 'c' || pressedKey == 'C') {
            engine.clearRecent();
            selectedIndex = 0;
            scrollOffset = 0;
            dirty = true;
            displaySuccess("Cleared List", true);
            rf_clear_nav_state();
        }

        if (check(PrevPress) || check(UpPress)) {
            if (selectedIndex > 0) {
                selectedIndex--;
                if (selectedIndex < scrollOffset) {
                    scrollOffset = selectedIndex;
                }
                dirty = true;
            }
        }
        if (check(NextPress) || check(DownPress)) {
            size_t count = engine.getRecentCount();
            if (count > 0 && selectedIndex < (int)count - 1) {
                selectedIndex++;
                int maxLines = (tftHeight - 55) / (FP * LH + 1);
                if (maxLines < 1) maxLines = 1;
                if (selectedIndex >= scrollOffset + maxLines) {
                    scrollOffset = selectedIndex - maxLines + 1;
                }
                dirty = true;
            }
        }

        if (check(SelPress)) {
            size_t count = engine.getRecentCount();
            if (count > 0) {
                rx.end();
                int targetIdx = (int)count - 1 - selectedIndex;
                if (targetIdx < 0) targetIdx = 0;
                if (targetIdx >= (int)count) targetIdx = (int)count - 1;
                show_reading_details(targetIdx);

                // Restart reception. The CC1101 was never deinitialized while viewing the
                // details screen (only the RMT capture session was torn down via rx.end()
                // above), so re-arm the capture without re-running engine.initRadio() —
                // that would needlessly reconfigure the radio (incl. a setMHZ() frequency
                // re-write) even though frequency/preset never changed.
                rx.begin();
                rx.flush();
                if (isHopping) hopStart = millis();
                rf_clear_nav_state();
                dirty = true;
            }
        }

        if (dirty) {
            dirty = false;
            if (isHopping) {
                uint32_t elapsed = millis() - hopStart;
                uint32_t remSec = (elapsed >= engine.hopTimeoutMs) ? 0 : ((engine.hopTimeoutMs - elapsed + 999) / 1000);
                String headerTitle = "SCAN [" + String(remSec) + "s] " + String(currentFreq, 2) + "M " +
                                     String(rtl433_get_preset_name(currentPreset));
                drawMainBorderWithTitle(headerTitle);
            } else {
                String headerTitle = "RTL433: " + String(currentFreq, 2) + "M " +
                                     String(rtl433_get_preset_name(currentPreset));
                drawMainBorderWithTitle(headerTitle);
            }

            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.setTextSize(FP);
            setPadCursor(1, 0);

            padprintln("Rx: " + String(engine.getPacketsReceived()) +
                       "  Dec: " + String(engine.getPacketsDecoded()) +
                       "  RSSI: " + String(currentRssi) + "dBm");

            size_t count = engine.getRecentCount();
            if (count == 0) {
                tft.setTextColor(getColorVariation(bruceConfig.priColor), bruceConfig.bgColor);
                if (isHopping) {
                    padprintln("\n  Scanning profile...\n  (" + String(rtl433_get_changing_preset_name(engine.changingPreset)) + ")");
                } else {
                    padprintln("\n  Listening on " + String(currentFreq, 2) + "M\n  (" + String(rtl433_get_preset_name(currentPreset)) + ")");
                }
            } else {
                int maxLines = (tftHeight - 55) / (FP * LH + 1);
                if (maxLines < 1) maxLines = 1;

                int maxScroll = (int)count - maxLines;
                if (maxScroll < 0) maxScroll = 0;
                if (scrollOffset > maxScroll) scrollOffset = maxScroll;
                if (scrollOffset < 0) scrollOffset = 0;

                for (int i = 0; i < maxLines && (scrollOffset + i) < (int)count; i++) {
                    int itemRel = scrollOffset + i;
                    int storageIdx = (int)count - 1 - itemRel;
                    const Rtl433Reading *r = engine.getRecentAt(storageIdx);
                    if (!r) continue;

                    if (itemRel == selectedIndex) {
                        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
                        padprintln("> " + r->toSummaryLine());
                    } else {
                        tft.setTextColor(getColorVariation(bruceConfig.priColor), bruceConfig.bgColor);
                        padprintln("  " + r->toSummaryLine());
                    }
                }
            }

            tft.setTextColor(getColorVariation(bruceConfig.priColor), bruceConfig.bgColor);
            tft.drawCentreString(
                "[OK] View  [C] Clear  [ESC] Exit", tftWidth / 2, tftHeight - BORDER_PAD_X - FP * LH, SMOOTH_FONT
            );
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    rx.end();
    engine.deinitRadio();
    rf_clear_nav_state();
}

void rtl433_menu() {
    Rtl433Engine &engine = Rtl433Engine::instance();
    bool exitMain = false;

    while (!exitMain) {
        String profileLabel = engine.isChangingPreset ?
            ("Presets: [" + String(rtl433_get_changing_preset_name(engine.changingPreset)) + "]") :
            ("Presets: [" + String(rtl433_get_preset_name(engine.currentPreset)) + "]");

        std::vector<Option> opts = {
            {"Sniff Live",       []() { rtl433_sniff_screen(); }                   },
            {profileLabel,       rtl433_presets_menu                               },
            {"Recent Packets",   view_recent_packets_menu                          },
            {"Clear Results",    [&]() {
                engine.clearRecent();
                displaySuccess("Cleared List", true);
            }},
            {"Replay Settings",  rtl433_replay_menu                                },
            {"Test Transmit",    rtl433_test_tx_menu                               },
            {String("SD Logging: ") + (engine.sdLoggingEnabled ? "[ON]" : "[OFF]"), [&]() {
                engine.sdLoggingEnabled = !engine.sdLoggingEnabled;
                displayInfo(String("SD Logging: ") + (engine.sdLoggingEnabled ? "ON" : "OFF"), true);
            }},
            {"View SD Log",      view_sd_log_menu                                  },
            {"Decoders Info",    show_decoders_info                                },
            {"Go Back",          [&]() { exitMain = true; }                        },
        };

        int res = loopOptions(opts, MENU_TYPE_SUBMENU, "RTL433 Receiver");
        if (check(EscPress) || res < 0 || returnToMenu || exitMain) {
            returnToMenu = false;
            break;
        }
    }

    engine.clearRecent();
}
