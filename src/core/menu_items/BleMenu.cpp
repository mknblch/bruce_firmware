#include "BleMenu.h"
#include "core/display.h"
#include "core/utils.h"
#include "modules/badusb_ble/ducky_typer.h"
#include "modules/ble/ble_common.h"
#include "modules/ble/ble_ninebot.h"
#include "modules/ble/ble_spam.h"
#include "modules/ble/ble_tracker.h"
#if !defined(LITE_VERSION)
#include "modules/ble/BLE_Suite.h"
#include "modules/ble/gatt_explorer.h"
#include "modules/ble/gatt_server.h"
#include "modules/ble/race_client.h"
#else
#include "modules/ble/ble_sniffer.h"
#endif
#include <globals.h>

void BleMenu::optionsMenu() {
    returnToMenu = false;
    while (true) {
        if (returnToMenu) {
            returnToMenu = false;
            return;
        }

        std::vector<Option> localOptions;
#if !defined(LITE_VERSION)
        if (BLEConnected) {
            localOptions.push_back({"Disconnect", [=]() {
                                       BLEDevice::deinit();
                                       BLEConnected = false;
                                       delete hid_ble;
                                       hid_ble = nullptr;
                                   }});
        }
        localOptions.push_back({"Media Cmds", [=]() { MediaCommands(hid_ble, true); }});
        localOptions.push_back({"BLE Scan", ble_scan});
        localOptions.push_back({"BLE Tracker", [=]() { BleTrackerMenu(); }});
        localOptions.push_back({"GATT Explorer", gattExplorerMenu});
        localOptions.push_back({"GATT Server", gattServerMenu});
        localOptions.push_back({"RACE Client", raceMainMenu});
        localOptions.push_back({"iBeacon", [=]() {
                                   ibeacon("Bruce", "e4c159a0-8c82-11e6-bdf4-0800200c9a66", 0x004C);
                               }});
        localOptions.push_back({"Bad BLE", [=]() { ducky_setup(hid_ble, true); }});
        localOptions.push_back({"BLE Keyboard", [=]() { ducky_keyboard(hid_ble, true); }});
#endif
        localOptions.push_back({"BLE Spam", [=]() { spamMenu(); }});

#if !defined(LITE_VERSION)
        localOptions.push_back({"BLE Suite", [=]() { BleSuiteMenu(); }});
        localOptions.push_back({"Ninebot", [=]() { BLENinebot(); }});
        localOptions.push_back({"Presenter mode", [=]() { PresenterMode(hid_ble, true); }});
#else
        localOptions.push_back({"BLE Sniffer", [=]() { BLE_SnifferMenu(); }});
        localOptions.push_back({"BLE Tracker", [=]() { BleTrackerMenu(); }});
#endif
        localOptions.push_back({"Main Menu", []() {}});

        int selected = loopOptions(localOptions, MENU_TYPE_SUBMENU, "Bluetooth", 0, false);
        if (selected == -1 || selected == (int)localOptions.size() - 1) {
            return;
        }
    }
}

void BleMenu::drawIcon(float scale) {
    clearIconArea();

    int lineWidth = scale * 5;
    int iconW = scale * 36;
    int iconH = scale * 60;
    int radius = scale * 5;
    int deltaRadius = scale * 10;

    if (iconW % 2 != 0) iconW++;
    if (iconH % 4 != 0) iconH += 4 - (iconH % 4);

    tft.drawWideLine(
        iconCenterX,
        iconCenterY + iconH / 4,
        iconCenterX - iconW,
        iconCenterY - iconH / 4,
        lineWidth,
        bruceConfig.priColor,
        bruceConfig.priColor
    );
    tft.drawWideLine(
        iconCenterX,
        iconCenterY - iconH / 4,
        iconCenterX - iconW,
        iconCenterY + iconH / 4,
        lineWidth,
        bruceConfig.priColor,
        bruceConfig.priColor
    );
    tft.drawWideLine(
        iconCenterX,
        iconCenterY + iconH / 4,
        iconCenterX - iconW / 2,
        iconCenterY + iconH / 2,
        lineWidth,
        bruceConfig.priColor,
        bruceConfig.priColor
    );
    tft.drawWideLine(
        iconCenterX,
        iconCenterY - iconH / 4,
        iconCenterX - iconW / 2,
        iconCenterY - iconH / 2,
        lineWidth,
        bruceConfig.priColor,
        bruceConfig.priColor
    );

    tft.drawWideLine(
        iconCenterX - iconW / 2,
        iconCenterY - iconH / 2,
        iconCenterX - iconW / 2,
        iconCenterY + iconH / 2,
        lineWidth,
        bruceConfig.priColor,
        bruceConfig.priColor
    );

    tft.drawArc(
        iconCenterX,
        iconCenterY,
        2.5 * radius,
        2 * radius,
        210,
        330,
        bruceConfig.priColor,
        bruceConfig.bgColor
    );
    tft.drawArc(
        iconCenterX,
        iconCenterY,
        2.5 * radius + deltaRadius,
        2 * radius + deltaRadius,
        210,
        330,
        bruceConfig.priColor,
        bruceConfig.bgColor
    );
    tft.drawArc(
        iconCenterX,
        iconCenterY,
        2.5 * radius + 2 * deltaRadius,
        2 * radius + 2 * deltaRadius,
        210,
        330,
        bruceConfig.priColor,
        bruceConfig.bgColor
    );
}
