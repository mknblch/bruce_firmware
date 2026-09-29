#ifndef __LORA_RADIO_STATE_HELPERS_H__
#define __LORA_RADIO_STATE_HELPERS_H__

#include <cstdint>

namespace LoRaRadioStateHelpers {

inline bool hasAnyIrqFlag(uint32_t flags, uint32_t mask) {
    return (flags & mask) != 0;
}

inline bool receiveCanBeEnabled(bool configurationSucceeded, bool receiveStarted) {
    return configurationSucceeded && receiveStarted;
}

inline bool probeOwnsRadioResources(bool radioWasAlreadyInitialized) {
    return !radioWasAlreadyInitialized;
}

} // namespace LoRaRadioStateHelpers

#endif