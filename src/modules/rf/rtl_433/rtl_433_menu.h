// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "rtl_433.h"

// RTL433 Submenu entry point in RF Menu
void rtl433_menu();

// Sniffer screen (uses active fixed or changing preset from engine)
void rtl433_sniff_screen(bool hopping = false);

// Presets configuration menu (choose fixed or changing presets)
void rtl433_presets_menu();

// Replay settings and configuration menu
void rtl433_replay_menu();
