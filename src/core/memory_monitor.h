#ifndef __MEMORY_MONITOR_H__
#define __MEMORY_MONITOR_H__

#include <Arduino.h>

/**
 * Real-time graphical Memory & Heap monitor screen.
 * Displays internal SRAM (Used, Free, Total), largest allocatable block (Fragmentation),
 * peak stress watermark (Min Free Ever), PSRAM status, and live heap integrity testing.
 */
void showMemoryMonitor();

#endif // __MEMORY_MONITOR_H__
