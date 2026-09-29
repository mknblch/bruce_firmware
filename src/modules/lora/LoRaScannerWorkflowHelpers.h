#ifndef __LORA_SCANNER_WORKFLOW_HELPERS_H__
#define __LORA_SCANNER_WORKFLOW_HELPERS_H__

#include <cstdint>

namespace LoRaScannerWorkflowHelpers {

inline bool shouldPersistSelectedChannel(int actionIndex) {
    return actionIndex == 2;
}

inline uint32_t restartDwellTimer(uint32_t nowMs) {
    return nowMs;
}

inline bool shouldContinueAfterRadioTransition(bool transitionSucceeded) {
    return transitionSucceeded;
}

template <typename Config, typename Channel>
Config configForChannel(Config config, const Channel &channel, uint8_t fallbackCr, uint16_t fallbackPreambleLen) {
    config.freqMHz = channel.freqMHz;
    config.sf = channel.sf;
    config.bwKHz = channel.bwKHz;
    config.cr = channel.hasRadioParams ? channel.cr : fallbackCr;
    config.syncWord = channel.syncWord;
    config.preambleLen = channel.hasRadioParams ? channel.preambleLen : fallbackPreambleLen;
    return config;
}

} // namespace LoRaScannerWorkflowHelpers

#endif