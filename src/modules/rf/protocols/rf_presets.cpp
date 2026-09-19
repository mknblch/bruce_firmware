#include "rf_presets.h"
#include "../rf_utils.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <globals.h>

// Canonical preset table. Values reproduce exactly the parameters that used
// to be inlined in sendRfCommand(): a field at 0 means "keep module default".
//   default OOK radio params (module defaults): mod=2, dev=1.58, BW=270.83,
//   dataRate=10. So OOK presets only override what differs.
static const RfPreset rf_presets[] = {
    // name                         mod  dev        rxBW   dataRate  legacyProto defaultFreq label
    {"Ook270Async",                  2, 0.0f,       270.f, 0.0f,     1,          433.92f,    "433.92M OOK (270k)"},
    {"Ook650Async",                  2, 0.0f,       650.f, 0.0f,     2,          433.92f,    "433.92M OOK (650k)"},
    {"Ook270Async868",               2, 0.0f,       270.f, 0.0f,     1,          868.35f,    "868.35M OOK (270k)"},
    {"Ook270Async315",               2, 0.0f,       270.f, 0.0f,     1,          315.00f,    "315.00M OOK (270k)"},
    {"Ook270Async345",               2, 0.0f,       270.f, 0.0f,     1,          345.00f,    "345.00M OOK (Honeywell)"},
    {"2FSKDev238Async",              0, 2.380371f,  238.f, 0.0f,     1,          433.92f,    "433.92M 2-FSK (2.38k)"},
    {"2FSKDev476Async",              0, 47.60742f,  476.f, 0.0f,     1,          433.92f,    "433.92M 2-FSK (47.6k)"},
    {"2FSKDev238Async868",           0, 19.042969f, 135.f, 17.24f,   1,          868.35f,    "868.35M 2-FSK (17.2k)"},
    {"2FSKDev476Async315",           0, 47.60742f,  200.f, 19.20f,   1,          315.00f,    "315.00M 2-FSK (19.2k)"},
    {"GFSK17_24KbAsync",             1, 19.042969f, 135.f, 17.24f,   1,          433.92f,    "433.92M GFSK (17.2k)"},
    {"GFSK9_99KbAsync",              1, 19.042969f, 0.0f,  9.996f,   1,          868.35f,    "868.35M GFSK (9.99k)"},
    {"MSK99_97KbAsync",              4, 47.60742f,  0.0f,  99.97f,   1,          868.95f,    "868.95M MSK (wM-Bus T)"},
    {"MSK32_76KbAsync",              4, 0.0f,       135.f, 32.768f,  1,          868.30f,    "868.30M MSK (wM-Bus S)"},
};

// Alias map: legacy Furi preset names found in existing `.sub` files →
// canonical preset name above. Keeps old files working without touching the
// preset table. Documented in protocols/README.md.
struct PresetAlias {
    const char *alias;
    const char *canonical;
};
static const PresetAlias rf_preset_aliases[] = {
    {"FuriHalSubGhzPresetOok270Async",      "Ook270Async"    },
    {"FuriHalSubGhzPresetOok650Async",      "Ook650Async"    },
    {"FuriHalSubGhzPreset2FSKDev238Async",  "2FSKDev238Async"},
    {"FuriHalSubGhzPreset2FSKDev476Async",  "2FSKDev476Async"},
    {"FuriHalSubGhzPresetMSK99_97KbAsync",  "MSK99_97KbAsync"},
    {"FuriHalSubGhzPresetGFSK9_99KbAsync",  "GFSK9_99KbAsync"},
    {"433_OOK_270",                         "Ook270Async"    },
    {"433_OOK_650",                         "Ook650Async"    },
    {"868_OOK_270",                         "Ook270Async868" },
    {"315_OOK_270",                         "Ook270Async315" },
    {"345_OOK_270",                         "Ook270Async345" },
    {"433_FSK_2K",                          "2FSKDev238Async"},
    {"433_FSK_47K",                         "2FSKDev476Async"},
    {"868_FSK_17K",                         "2FSKDev238Async868"},
    {"315_FSK_19K",                         "2FSKDev476Async315"},
    {"433_GFSK_17K",                        "GFSK17_24KbAsync"},
    {"868_GFSK_10K",                        "GFSK9_99KbAsync"},
    {"868_MSK_T",                           "MSK99_97KbAsync"},
    {"868_MSK_S",                           "MSK32_76KbAsync"},
};

int rf_presets_count() {
    return sizeof(rf_presets) / sizeof(rf_presets[0]);
}

const RfPreset *rf_preset_at(int idx) {
    if (idx < 0 || idx >= rf_presets_count()) return nullptr;
    return &rf_presets[idx];
}

const RfPreset *rf_find_preset(const String &name) {
    // resolve a legacy alias to its canonical name first
    String wanted = name;
    for (const auto &a : rf_preset_aliases) {
        if (name == a.alias) {
            wanted = a.canonical;
            break;
        }
    }
    for (const auto &p : rf_presets) {
        if (wanted == p.name) return &p;
    }
    return nullptr;
}

void rf_apply_preset(const RfPreset *preset, float freq) {
    if (!preset) return;
    float targetFreq = (freq > 0.0f) ? freq : (preset->defaultFreq > 0.0f ? preset->defaultFreq : bruceConfigPins.rfFreq);
    bruceConfigPins.setRfFreq(targetFreq, 1);
    if (bruceConfigPins.rfModule == CC1101_SPI_MODULE) {
        ELECHOUSE_cc1101.setSidle();
        ELECHOUSE_cc1101.setModulation(preset->modulation);
        if (preset->modulation != 2) {
            if (preset->deviation > 0.0f) ELECHOUSE_cc1101.setDeviation(preset->deviation);
            if (preset->rxBW > 0.0f) ELECHOUSE_cc1101.setRxBW(preset->rxBW);
            if (preset->dataRate > 0.0f) ELECHOUSE_cc1101.setDRate(preset->dataRate);
        } else {
            if (preset->rxBW > 0.0f) ELECHOUSE_cc1101.setRxBW(preset->rxBW);
        }
        setMHZ(targetFreq);
        ELECHOUSE_cc1101.SetRx();
    }
}
