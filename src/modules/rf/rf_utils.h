#ifndef __RF_UTILS_H__
#define __RF_UTILS_H__

#include "protocols/rf_keeloq.h" // KeeLoq cipher/keystore + bitAt/g5/KEELOQ_* macros
#include "structs.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>
// ESP-IDF 5.5 based framework determines the channels autommatically
// you do not have the hability to choose the channel
rmt_channel_handle_t setup_rf_rx();

#define RMT_MAX_PULSES 10000 // Maximum number of pulses to record
#define RMT_CLK_DIV 80       /*!< RMT counter clock divider */
#define RMT_1US_TICKS (80000000 / RMT_CLK_DIV / 1000000)
#define RMT_1MS_TICKS (RMT_1US_TICKS * 1000)
#define SIGNAL_STRENGTH_THRESHOLD 1500 // Adjust this threshold as needed

extern const float subghz_frequency_list[93];
extern const char *subghz_frequency_ranges[];
extern const int range_limits[4][2];
extern bool rmtInstalled;

// modulation: -1 = unspecified (defaults to OOK/ASK for backward compatibility), otherwise
// the CC1101 modulation code (0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK). When a
// non-OOK modulation is requested, the FSK-family fixed-frequency register/AGC preset is
// applied instead of the OOK-tuned one (see cc1101ApplyFixedFreqFskPreset()).
bool initRfModule(
    String mode = "", float frequency = 0, int modulation = -1, float deviation = 0.0f, float rxBw = 0.0f,
    float dataRate = 0.0f
);
void deinitRfModule();
void initCC1101once(SPIClass *SSPI);

// Fixed-frequency CC1101 register/AGC presets applied by initRfModule() (and re-applied
// directly by callers such as Rtl433Engine::switchPreset() that swap presets without a full
// re-init). Keep the OOK preset untouched for OOK/ASK use; use the FSK preset for
// 2-FSK/GFSK/MSK so non-OOK sessions don't inherit OOK-tuned AGC/bandwidth settings.
void cc1101ApplyFixedFreqOokPreset(bool isTx);
void cc1101ApplyFixedFreqFskPreset(bool isTx);

void setMHZ(float frequency);
int find_pulse_index(const std::vector<int> &indexed_durations, int duration);
uint64_t crc64_ecma(const std::vector<int> &data);

void addToRecentCodes(struct RfCodes rfcode);
struct RfCodes selectRecentRfMenu();
bool setMHZMenu();
void rf_range_selection(float currentFrequency = 0.0);
void selectRFPresetMenu();

uint64_t reverse_bits(uint64_t num, uint8_t bits);

String rf_subghz_header(float frequencyMHz);

#endif
