#include "ble_commands.h"
#include "modules/ble/gatt_server.h"
#include "modules/ble/gatt_explorer.h"
#include "modules/ble/race_client.h"
#include "modules/ble/ble_oui.h"
#include <globals.h>

#if !defined(LITE_VERSION)

static uint32_t bleCallback(cmd *c) {
    Command cmd(c);
    Argument actionArg = cmd.getArgument("action");
    String action = actionArg.getValue();
    action.trim();
    action.toLowerCase();

    Argument param1Arg = cmd.getArgument("param1");
    String param1 = param1Arg.getValue();
    param1.trim();

    Argument param2Arg = cmd.getArgument("param2");
    String param2 = param2Arg.getValue();
    param2.trim();

    Argument param3Arg = cmd.getArgument("param3");
    String param3 = param3Arg.getValue();
    param3.trim();

    Argument param4Arg = cmd.getArgument("param4");
    String param4 = param4Arg.getValue();
    param4.trim();

    Argument param5Arg = cmd.getArgument("param5");
    String param5 = param5Arg.getValue();
    param5.trim();

    if (action == "server") {
        if (param1 == "stop") {
            stopGattServerService();
            serialDevice->println("GATT Server stopped.");
            return true;
        } else if (param1 == "start" || param1 == "on" || param1 == "") {
            int profile = param2.toInt();
            if (profile < 0 || profile > 3) profile = 0;
            if (startGattServerService(profile)) {
                serialDevice->println("GATT Server started in profile " + String(profile));
                return true;
            } else {
                serialDevice->println("Failed to start GATT Server.");
                return false;
            }
        } else if (param1 == "status") {
            serialDevice->println(String("GATT Server status: ") + (isGattServerActive() ? "RUNNING" : "STOPPED"));
            return true;
        }
    } else if (action == "connect") {
        if (param1 == "") {
            serialDevice->println("Usage: ble connect <MAC> [pub|rnd]");
            return false;
        }
        uint8_t addrType = 0; // BLE_ADDR_PUBLIC
        if (param2.equalsIgnoreCase("rnd") || param2.equalsIgnoreCase("random") || param2 == "1") {
            addrType = 1; // BLE_ADDR_RANDOM
        }
        bool ok = gattConnectCli(param1, addrType);
        return ok;
    } else if (action == "race") {
        if (param1 == "") {
            serialDevice->println("Usage: ble race <MAC> [pub|rnd] <check|info|ram|flash|parttable|raw> [args...]");
            return false;
        }
        uint8_t addrType = 0;
        String subCmd = "";
        String arg1 = "";
        String arg2 = "";
        String arg3 = "";

        if (param2.equalsIgnoreCase("rnd") || param2.equalsIgnoreCase("random") || param2 == "1") {
            addrType = 1;
            subCmd = param3;
            arg1 = param4;
            arg2 = param5;
        } else if (param2.equalsIgnoreCase("pub") || param2.equalsIgnoreCase("public") || param2 == "0") {
            addrType = 0;
            subCmd = param3;
            arg1 = param4;
            arg2 = param5;
        } else {
            // param2 is the subcommand directly
            addrType = 0;
            subCmd = param2;
            arg1 = param3;
            arg2 = param4;
            arg3 = param5;
        }

        if (subCmd.isEmpty()) subCmd = "check";
        return raceCli(param1, addrType, subCmd, arg1, arg2, arg3);
    } else if (action == "scan") {
        int timeoutSec = param1.toInt();
        if (timeoutSec <= 0) timeoutSec = 5;
        gattScanCli(timeoutSec);
        return true;
    } else if (action == "oui") {
        if (param1 == "" || param1 == "status" || param1 == "info") {
            serialDevice->println("=== OUI & Vendor Database Status ===");
            serialDevice->printf("SD Database Available: %s\n", isSdOuiDatabaseAvailable() ? "YES" : "NO");
            serialDevice->printf("SD Card Mounted:       %s\n", sdcardMounted ? "YES" : "NO");
            serialDevice->println("Usage: ble oui <MAC_or_OUI_hex> (e.g. ble oui AC:67:84:11:22:33, ble oui 001122, ble oui AC6784)");
            return true;
        }
        serialDevice->println("[BLE-CLI] Testing OUI resolution for: " + param1);
        String clean = param1;
        clean.replace(":", "");
        clean.replace("-", "");
        clean.replace("0x", "");
        clean.replace("0X", "");
        if (clean.length() >= 6) {
            uint32_t oui = (uint32_t)strtoul(clean.substring(0, 6).c_str(), NULL, 16);
            serialDevice->printf("[BLE-CLI] Parsed OUI: 0x%06X\n", (unsigned int)oui);
            String vendor = resolveBleOui(oui, true);
            if (vendor.length() > 0) {
                serialDevice->println("[BLE-CLI] Result: Vendor = \"" + vendor + "\"");
            } else {
                serialDevice->println("[BLE-CLI] Result: Vendor = NOT FOUND (Unknown OUI)");
            }
            return true;
        } else {
            serialDevice->println("[BLE-CLI] Invalid OUI / MAC length (need at least 6 hex chars)");
            return false;
        }
    }

    serialDevice->println(
        "Invalid ble command.\n"
        "Usage:\n"
        "  ble server start [0-3]   (0=All-in-One, 1=DIS+Bat, 2=Echo, 3=NUS)\n"
        "  ble server stop\n"
        "  ble server status\n"
        "  ble scan [seconds]\n"
        "  ble connect <MAC> [pub|rnd]\n"
        "  ble race <MAC> [pub|rnd] <check|info|media|ram|flash|parttable|raw>\n"
        "  ble oui [status|<MAC|OUI>]"
    );
    return false;
}

void createBleCommands(SimpleCLI *cli) {
    Command bleCmd = cli->addCommand("ble", bleCallback);
    bleCmd.addPosArg("action", "");
    bleCmd.addPosArg("param1", "");
    bleCmd.addPosArg("param2", "");
    bleCmd.addPosArg("param3", "");
    bleCmd.addPosArg("param4", "");
    bleCmd.addPosArg("param5", "");
}

#else

void createBleCommands(SimpleCLI *cli) {}

#endif // !LITE_VERSION
