// SPDX-License-Identifier: AGPL-3.0-or-later
#include "rtl_433_menu.h"
#include "core/display.h"
#include "core/led_control.h"
#include "core/mykeyboard.h"
#include "core/sd_functions.h"
#include "core/settings.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>

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
            "[OK] Menu   [ESC] Back", tftWidth / 2, tftHeight - BORDER_PAD_X - FP * LH, SMOOTH_FONT
        );

        rf_clear_nav_state();
        delay(100);

        while (1) {
            if (check(EscPress)) {
                exitView = true;
                break;
            }
            if (check(SelPress)) {
                enum Action { ACT_NONE, ACT_REPLAY, ACT_SAVE, ACT_DUMP, ACT_CLEAR, ACT_BACK };
                Action chosenAction = ACT_NONE;

                std::vector<Option> opts = {
                    {"Replay RF",     [&]() { chosenAction = ACT_REPLAY; }},
                    {"Save to .SUB",  [&]() { chosenAction = ACT_SAVE; }},
                    {"Dump JSON",     [&]() { chosenAction = ACT_DUMP; }},
                    {"Clear Results", [&]() { chosenAction = ACT_CLEAR; }},
                    {"Go Back",       [&]() { chosenAction = ACT_BACK; }},
                };

                loopOptions(opts, MENU_TYPE_SUBMENU, r.protocol.c_str());

                switch (chosenAction) {
                    case ACT_REPLAY:
                        displayTextLine("Replaying RF...");
                        engine.replayReading(r);
                        delay(600);
                        break;
                    case ACT_SAVE: {
                        String savedPath;
                        if (engine.saveSubFile(r, &savedPath)) {
                            displaySuccess("Saved: " + savedPath, true);
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

static void select_preset_menu() {
    Rtl433Engine &engine = Rtl433Engine::instance();
    std::vector<Option> opts;

    for (int p = 0; p < RTL433_PRESET_COUNT; p++) {
        const Rtl433PresetDef *pdef = rtl433_get_preset_def(p);
        String label = String(pdef->name) + " (" + String(pdef->default_freq, 2) + "M)";
        opts.push_back({label, [p, &engine]() {
            engine.currentPreset = p;
            engine.currentFrequency = rtl433_get_preset_def(p)->default_freq;
            displaySuccess("Set: " + String(rtl433_get_preset_name(p)), true);
        }});
    }
    opts.push_back({"Go Back", []() {}});
    loopOptions(opts, engine.currentPreset);
}

static void select_frequency_menu() {
    Rtl433Engine &engine = Rtl433Engine::instance();
    std::vector<Option> opts = {
        {"433.92 MHz", [&]() { engine.currentFrequency = 433.92f; }},
        {"868.35 MHz", [&]() { engine.currentFrequency = 868.35f; }},
        {"868.95 MHz (wM-Bus T)", [&]() { engine.currentFrequency = 868.95f; }},
        {"868.30 MHz (wM-Bus S)", [&]() { engine.currentFrequency = 868.30f; }},
        {"915.00 MHz", [&]() { engine.currentFrequency = 915.00f; }},
        {"345.00 MHz (Honeywell)", [&]() { engine.currentFrequency = 345.00f; }},
        {"315.00 MHz (TPMS)", [&]() { engine.currentFrequency = 315.00f; }},
        {"434.42 MHz", [&]() { engine.currentFrequency = 434.42f; }},
    };
    opts.push_back({"Go Back", []() {}});
    loopOptions(opts, MENU_TYPE_SUBMENU, "Select Frequency");
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
            displaySuccess(String("Timeout: ") + label, true);
        }});
    }
    opts.push_back({"Go Back", []() {}});
    loopOptions(opts, MENU_TYPE_SUBMENU, "Hop Timeout");
}

static void select_hop_group_menu() {
    Rtl433Engine &engine = Rtl433Engine::instance();
    std::vector<Option> opts;
    for (int g = 0; g < RTL433_HOP_GROUP_COUNT; g++) {
        const char *name = rtl433_get_hop_group_name(g);
        opts.push_back({name, [&engine, g, name]() {
            engine.hopGroup = g;
            displaySuccess(String("Group: ") + name, true);
        }});
    }
    opts.push_back({"Go Back", []() {}});
    loopOptions(opts, engine.hopGroup);
}

void rtl433_hop_menu() {
    Rtl433Engine &engine = Rtl433Engine::instance();
    bool exitHop = false;
    while (!exitHop) {
        String timeoutStr = String(engine.hopTimeoutMs / 1000) + "s";
        std::vector<Option> opts = {
            {"Start Hop Sniff",   []() { rtl433_sniff_screen(true); }                                           },
            {"Hop Timeout (" + timeoutStr + ")", select_hop_timeout_menu                                         },
            {"Hop Band (" + String(rtl433_get_hop_group_name(engine.hopGroup)) + ")", select_hop_group_menu   },
            {String("Extend on Hit: ") + (engine.hopStayOnSignal ? "[ON]" : "[OFF]"), [&]() {
                engine.hopStayOnSignal = !engine.hopStayOnSignal;
                displayInfo(String("Extend on Hit: ") + (engine.hopStayOnSignal ? "ON" : "OFF"), true);
            }},
            {"Go Back",           [&]() { exitHop = true; }                                                     },
        };
        int res = loopOptions(opts, MENU_TYPE_SUBMENU, "RTL433 Hopping");
        if (check(EscPress) || res < 0 || returnToMenu || exitHop) {
            returnToMenu = false;
            break;
        }
    }
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

void rtl433_sniff_screen(bool hopping) {
    Rtl433Engine &engine = Rtl433Engine::instance();

    std::vector<int> hopList;
    size_t currentHopIdx = 0;
    int currentPreset = engine.currentPreset;
    float currentFreq = engine.currentFrequency;

    if (hopping) {
        hopList = rtl433_get_hop_presets(engine.hopGroup);
        if (hopList.empty()) hopList.push_back(RTL433_PRESET_OOK_433);
        currentPreset = hopList[0];
        currentFreq = rtl433_get_preset_def(currentPreset)->default_freq;
    }

    if (!engine.initRadio(currentFreq, currentPreset)) {
        displayError("Radio Init Failed", true);
        return;
    }

    RfRxSession rx;
    if (!rx.begin()) {
        engine.deinitRadio();
        displayError("RX Session Failed", true);
        return;
    }

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
        if (hopping) {
            uint32_t elapsed = now - hopStart;
            if (elapsed >= engine.hopTimeoutMs) {
                currentHopIdx = (currentHopIdx + 1) % hopList.size();
                currentPreset = hopList[currentHopIdx];
                currentFreq = rtl433_get_preset_def(currentPreset)->default_freq;
                engine.switchPreset(currentFreq, currentPreset);
                hopStart = millis();
                dirty = true;
            } else if (now - lastTimerUpdate >= 1000) {
                lastTimerUpdate = now;
                uint32_t remSec = (elapsed >= engine.hopTimeoutMs) ? 0 : ((engine.hopTimeoutMs - elapsed + 999) / 1000);
                String headerTitle = "HOP [" + String(remSec) + "s] " + String(currentFreq, 2) + "M " +
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

        std::vector<int> durations;
        if (rx.poll(durations)) {
            Rtl433Reading reading;
            if (engine.decode(durations, currentFreq, currentPreset, currentRssi, reading)) {
                blinkLed();
                engine.addRecent(reading);
                engine.logJson(reading, engine.sdLoggingEnabled);
                if (hopping && engine.hopStayOnSignal) {
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

                // Restart reception
                engine.initRadio(currentFreq, currentPreset);
                rx.begin();
                if (hopping) hopStart = millis();
                rf_clear_nav_state();
                dirty = true;
            }
        }

        if (dirty) {
            dirty = false;
            if (hopping) {
                uint32_t elapsed = millis() - hopStart;
                uint32_t remSec = (elapsed >= engine.hopTimeoutMs) ? 0 : ((engine.hopTimeoutMs - elapsed + 999) / 1000);
                String headerTitle = "HOP [" + String(remSec) + "s] " + String(currentFreq, 2) + "M " +
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
                if (hopping) {
                    padprintln("\n  Hopping modulations...\n  (Listening on " + String(currentFreq, 2) + "M)");
                } else {
                    padprintln("\n  Listening for sensors...\n  (Weather, TPMS, Alarms)");
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
        std::vector<Option> opts = {
            {"Sniff Live",       []() { rtl433_sniff_screen(false); }              },
            {"Hop Sniffer",      []() { rtl433_sniff_screen(true); }               },
            {"Hop Settings",     rtl433_hop_menu                                   },
            {"Replay Settings",  rtl433_replay_menu                                },
            {"Recent Packets",   view_recent_packets_menu                          },
            {"Clear Results",    [&]() {
                engine.clearRecent();
                displaySuccess("Cleared List", true);
            }},
            {"Modulation",       select_preset_menu                                },
            {"Frequency",        select_frequency_menu                             },
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
}
