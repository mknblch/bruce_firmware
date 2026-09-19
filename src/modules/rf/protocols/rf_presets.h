#pragma once

#include "rf_protocol.h"

// Radio preset registry. Single place where the `.sub` "Preset:" names map
// to concrete transceiver parameters.

// Total number of built-in presets
int rf_presets_count();

// Get preset by index
const RfPreset *rf_preset_at(int idx);

// Resolve a preset by its canonical/alias name. Returns nullptr if unknown
// (caller then tries the numeric legacy-protocol path).
const RfPreset *rf_find_preset(const String &name);

// Apply a preset (frequency + modulation, deviation, bandwidth, data rate)
void rf_apply_preset(const RfPreset *preset, float freq = 0.0f);
