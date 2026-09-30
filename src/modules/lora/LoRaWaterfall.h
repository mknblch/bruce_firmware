#ifndef __LORA_WATERFALL_H__
#define __LORA_WATERFALL_H__

#if !defined(LITE_VERSION)
#include <Arduino.h>

extern float gLoraWaterfallStartFreq;
extern float gLoraWaterfallEndFreq;

void runLoRaWaterfallMenu();
void runLoRaWaterfall();
void runLoRaWaterfallWithSpan(float startFreqMHz, float endFreqMHz);

#endif // !LITE_VERSION
#endif // __LORA_WATERFALL_H__
