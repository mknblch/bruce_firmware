#ifndef __LORA_TRACKER_H__
#define __LORA_TRACKER_H__

#if !defined(LITE_VERSION)
#include <Arduino.h>

void runLoRaTrackerMenu();
void trackLoRaTarget(const String &targetMacOrId, const String &label = "");

#endif // !LITE_VERSION
#endif // __LORA_TRACKER_H__
