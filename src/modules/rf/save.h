#ifndef RF_SAVE_H
#define RF_SAVE_H
#include "structs.h"

bool rf_raw_save(RawRecording recorded);
bool rf_raw_save_durations(const std::vector<int> &durations, float frequency, String *outFilename = nullptr);
#endif
