#include "power_commands.h"
#include "core/settings.h"
#include "core/utils.h"
#include <globals.h>

uint32_t poweroffCallback(cmd *c) {
    powerOff();
    esp_deep_sleep_start(); // only wake up via hardware reset
    return true;
}

uint32_t rebootCallback(cmd *c) {
    ESP.restart();
    return true;
}

uint32_t sleepCallback(cmd *c) {
    setSleepMode();
    return true;
}

uint32_t batteryCallback(cmd *c) {
    int level = getBattery();
    uint32_t vbat = getBatteryVoltage();
    uint32_t vadc = getBatteryAdcMilliVolts();
    serialDevice->printf("[BATTERY] Level: %d%% | Voltage: %u mV | ADC: %u mV\n", level, (unsigned int)vbat, (unsigned int)vadc);
    return true;
}

void createPoweroffCommand(SimpleCLI *cli) { Command cmd = cli->addCommand("poweroff", poweroffCallback); }

void createRebootCommand(SimpleCLI *cli) { Command cmd = cli->addCommand("reboot", rebootCallback); }

void createSleepCommand(SimpleCLI *cli) { Command cmd = cli->addCommand("sleep", sleepCallback); }

void createBatteryCommand(SimpleCLI *cli) {
    cli->addCommand("battery", batteryCallback);
    cli->addCommand("bat", batteryCallback);
}

void createPowerCommand(SimpleCLI *cli) {
    Command cmd = cli->addCompositeCommand("power");

    Command cmdOff = cmd.addCommand("off", poweroffCallback);
    Command cmdReboot = cmd.addCommand("reboot", rebootCallback);
    Command cmdSleep = cmd.addCommand("sleep", sleepCallback);
    Command cmdBat = cmd.addCommand("battery", batteryCallback);
    Command cmdBatShort = cmd.addCommand("bat", batteryCallback);
}

void createPowerCommands(SimpleCLI *cli) {
    createPoweroffCommand(cli);
    createRebootCommand(cli);
    createSleepCommand(cli);
    createBatteryCommand(cli);

    createPowerCommand(cli);
}
