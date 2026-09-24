#if !defined(LITE_VERSION)
#include "LoRaMenu.h"
#include "core/configPins.h"
#include "core/display.h"
#include "core/settings.h"
#include "core/utils.h"
#include "modules/lora/LoRaConfig.h"
#include "modules/lora/LoRaRadio.h"
#include "modules/lora/LoRaRF.h"
#include "modules/lora/LoRaScanner.h"
#include "modules/lora/LoRaSniffer.h"
#include "modules/lora/LoRaTracker.h"

extern BruceConfigPins bruceConfigPins;

static bool verifyLoRaModuleGuard() {
    if (!isLoraHardwareConfigured()) {
        displayError("LoRa pins not configured!\nCheck Pin Setup", true);
        return false;
    }
    return true;
}

void LoRaMenu::optionsMenu() {
    loadLoRaConfig();
    bool hwConfigured = isLoraHardwareConfigured();

    options.clear();

    options.push_back({"Sniffer", []() {
        if (verifyLoRaModuleGuard()) runLoRaSniffer();
    }});

    options.push_back({"Channel Detector", []() {
        if (verifyLoRaModuleGuard()) runLoRaChannelDetector();
    }});

    options.push_back({"Signal Tracker", []() {
        if (verifyLoRaModuleGuard()) runLoRaTrackerMenu();
    }});

    options.push_back({"Chat", []() {
        if (verifyLoRaModuleGuard()) lorachat();
    }});

    options.push_back({"Presets", []() {
        selectLoRaPresetMenu();
    }});

    options.push_back({"Packet Viewer", []() {
        viewLoRaCapturedPackets();
    }});

    options.push_back({"Parameters / Config", []() {
        customLoRaConfigMenu();
    }});

    options.push_back({"Pin Setup", []() {
        setSPIPinsMenu(bruceConfigPins.LoRa_bus);
    }});

    addOptionToMainMenu();

    String txt = "LoRa (" + String(loraConfig.freqMHz, 2) + "M)";
    if (!hwConfigured) {
        txt = "LoRa [No Module]";
    }

    loopOptions(options, MENU_TYPE_SUBMENU, txt.c_str());
}

void LoRaMenu::drawIcon(float scale) {
    clearIconArea();
    scale *= 0.75;
    int cx = iconCenterX;
    int cy = iconCenterY + (scale * 8);

#define CALC_X(val) (cx + ((val - 50) * scale))
#define CALC_Y(val) (cy + ((val - 50) * scale))

    int lineWidth = scale * 4.5;
    if (lineWidth < 2) lineWidth = 2;
    int ballRad = scale * 6;

    // Left Leg
    tft.drawWideLine(
        CALC_X(44), CALC_Y(35), CALC_X(26), CALC_Y(85), lineWidth, bruceConfig.priColor, bruceConfig.priColor
    );
    // Right Leg
    tft.drawWideLine(
        CALC_X(56), CALC_Y(35), CALC_X(74), CALC_Y(85), lineWidth, bruceConfig.priColor, bruceConfig.priColor
    );

    // Top Cross Bar
    tft.drawWideLine(
        CALC_X(44), CALC_Y(35), CALC_X(56), CALC_Y(35), lineWidth, bruceConfig.priColor, bruceConfig.priColor
    );

    // Middle Cross Bar
    tft.drawWideLine(
        CALC_X(35), CALC_Y(65), CALC_X(65), CALC_Y(65), lineWidth, bruceConfig.priColor, bruceConfig.priColor
    );

    // X-Bracing (Top Section)
    tft.drawWideLine(
        CALC_X(44), CALC_Y(35), CALC_X(65), CALC_Y(65), lineWidth, bruceConfig.priColor, bruceConfig.priColor
    );
    tft.drawWideLine(
        CALC_X(56), CALC_Y(35), CALC_X(35), CALC_Y(65), lineWidth, bruceConfig.priColor, bruceConfig.priColor
    );

    // X-Bracing (Bottom Section)
    tft.drawWideLine(
        CALC_X(35), CALC_Y(65), CALC_X(74), CALC_Y(85), lineWidth, bruceConfig.priColor, bruceConfig.priColor
    );
    tft.drawWideLine(
        CALC_X(65), CALC_Y(65), CALC_X(26), CALC_Y(85), lineWidth, bruceConfig.priColor, bruceConfig.priColor
    );

    // Ball
    int ballY = CALC_Y(25);
    tft.fillCircle(cx, ballY, ballRad, bruceConfig.priColor);

    // Waves
    int r1 = scale * 20;
    int r2 = scale * 32;

    // Right Side
    tft.drawArc(cx, ballY, r1 + lineWidth, r1, 60, 120, bruceConfig.priColor, bruceConfig.bgColor);
    tft.drawArc(cx, ballY, r2 + lineWidth, r2, 60, 120, bruceConfig.priColor, bruceConfig.bgColor);

    // Left Side
    tft.drawArc(cx, ballY, r1 + lineWidth, r1, 240, 300, bruceConfig.priColor, bruceConfig.bgColor);
    tft.drawArc(cx, ballY, r2 + lineWidth, r2, 240, 300, bruceConfig.priColor, bruceConfig.bgColor);
}

#endif // !LITE_VERSION
