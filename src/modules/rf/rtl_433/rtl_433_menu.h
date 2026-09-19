// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "rtl_433.h"

// RTL433 Submenu entry point in RF Menu
void rtl433_menu();

// Sniffer screen (fixed single preset or auto-hopping)
void rtl433_sniff_screen(bool hopping = false);

// Hopping configuration and control menu
void rtl433_hop_menu();

// Replay settings and configuration menu
void rtl433_replay_menu();
