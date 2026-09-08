#include "memory_monitor.h"
#include "config.h"
#include "display.h"
#include "mykeyboard.h"
#include "utils.h"
#include <esp_heap_caps.h>

static void drawGaugeBar(int x, int y, int w, int h, float percent, uint16_t fgColor, uint16_t bgColor, uint16_t borderColor) {
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    tft.drawRoundRect(x, y, w, h, 2, borderColor);
    int innerW = w - 4;
    int innerH = h - 4;
    int fillW = (int)((float)innerW * (percent / 100.0f));
    if (fillW > 0) {
        tft.fillRect(x + 2, y + 2, fillW, innerH, fgColor);
    }
    if (innerW - fillW > 0) {
        tft.fillRect(x + 2 + fillW, y + 2, innerW - fillW, innerH, bgColor);
    }
}

void showMemoryMonitor() {
    tft.fillScreen(bruceConfig.bgColor);

    String integrityStatus = "Press [G] or [Enter] to test heap";
    uint16_t statusColor = TFT_DARKGREY;
    uint32_t lastCheckMillis = 0;
    uint32_t lastRedrawMillis = 0;

    int marginX = 8;
    int barW = tftWidth - 2 * marginX;
    if (barW < 20) barW = 20;

    const int barH = 9;

    // Draw footer once
    int footerY = tftHeight - LH * FP - 3;
    tft.fillRect(marginX, footerY, barW, LH * FP + 2, bruceConfig.bgColor);
    tft.setTextColor(TFT_DARKGREY, bruceConfig.bgColor);
    tft.setTextSize(FP);
    tft.drawString("[ESC] Exit    [G / Enter] Test Heap", marginX, footerY);

    while (true) {
        // Fast keyboard / button polling on each tick
        if (check(EscPress) || check(PrevPress)) {
            returnToMenu = true;
            break;
        }

        if (check(SelPress)) {
            bool ok = heap_caps_check_integrity_all(true);
            integrityStatus = ok ? "Heap Integrity: PASSED (OK)" : "Heap Integrity: CORRUPTED!";
            statusColor = ok ? TFT_GREEN : TFT_RED;
            lastCheckMillis = millis();
            lastRedrawMillis = 0; // Force immediate redraw
        }

#if defined(HAS_KEYBOARD)
        keyStroke key = _getKeyPress();
        if (key.pressed) {
            if (key.del) {
                returnToMenu = true;
                break;
            }
            if (key.enter) {
                bool ok = heap_caps_check_integrity_all(true);
                integrityStatus = ok ? "Heap Integrity: PASSED (OK)" : "Heap Integrity: CORRUPTED!";
                statusColor = ok ? TFT_GREEN : TFT_RED;
                lastCheckMillis = millis();
                lastRedrawMillis = 0; // Force immediate redraw
            }
            for (auto c : key.word) {
                if (c == 'q' || c == 'Q' || c == 0x1B || c == '`') {
                    returnToMenu = true;
                    break;
                }
                if (c == 'g' || c == 'G') {
                    bool ok = heap_caps_check_integrity_all(true);
                    integrityStatus = ok ? "Heap Integrity: PASSED (OK)" : "Heap Integrity: CORRUPTED!";
                    statusColor = ok ? TFT_GREEN : TFT_RED;
                    lastCheckMillis = millis();
                    lastRedrawMillis = 0; // Force immediate redraw
                }
            }
            if (returnToMenu) break;
        }
#endif

        if (lastCheckMillis > 0 && millis() - lastCheckMillis > 4000) {
            integrityStatus = "Press [G] or [Enter] to test heap";
            statusColor = TFT_DARKGREY;
            lastCheckMillis = 0;
            lastRedrawMillis = 0; // Force immediate redraw
        }

        // Redraw stats every 250ms or when forced
        uint32_t now = millis();
        if (lastRedrawMillis == 0 || now - lastRedrawMillis >= 250) {
            lastRedrawMillis = now;

            // Gather memory metrics
            size_t totalHeap = ESP.getHeapSize();
            size_t freeHeap = ESP.getFreeHeap();
            size_t minFreeHeap = ESP.getMinFreeHeap();
            size_t maxAllocHeap = ESP.getMaxAllocHeap();
            size_t usedHeap = totalHeap > freeHeap ? totalHeap - freeHeap : 0;
            size_t peakUsed = totalHeap > minFreeHeap ? totalHeap - minFreeHeap : 0;

            float usedPercent = totalHeap > 0 ? ((float)usedHeap / (float)totalHeap) * 100.0f : 0.0f;
            float fragPercent = freeHeap > 0 ? (1.0f - ((float)maxAllocHeap / (float)freeHeap)) * 100.0f : 0.0f;
            if (fragPercent < 0.0f) fragPercent = 0.0f;
            float contigPercent = freeHeap > 0 ? ((float)maxAllocHeap / (float)freeHeap) * 100.0f : 0.0f;

            size_t dmaFree = heap_caps_get_free_size(MALLOC_CAP_DMA);

            uint16_t heapBarColor = usedPercent > 85.0f ? TFT_RED : (usedPercent > 70.0f ? TFT_YELLOW : TFT_GREEN);
            uint16_t borderColor = bruceConfig.priColor;
            uint16_t barBgColor = TFT_BLACK;

            int y = 4;

            // 1. Internal Heap Header & Bar
            tft.setTextSize(FP);
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.fillRect(marginX, y, barW, LH * FP, bruceConfig.bgColor);
            String heapStr = "Heap: " + formatBytes(freeHeap) + " / " + formatBytes(totalHeap) + " (" + String((int)usedPercent) + "% used)";
            tft.drawString(heapStr, marginX, y);
            y += LH * FP + 1;

            drawGaugeBar(marginX, y, barW, barH, usedPercent, heapBarColor, barBgColor, borderColor);
            y += barH + 4;

            // 2. Max Contiguous Block & Fragmentation
            tft.setTextColor(TFT_CYAN, bruceConfig.bgColor);
            tft.fillRect(marginX, y, barW, LH * FP, bruceConfig.bgColor);
            String contigStr = "Max Block: " + formatBytes(maxAllocHeap) + "  Frag: " + String((int)fragPercent) + "%";
            tft.drawString(contigStr, marginX, y);
            y += LH * FP + 1;

            drawGaugeBar(marginX, y, barW, barH, contigPercent, TFT_CYAN, barBgColor, borderColor);
            y += barH + 4;

            // 3. Peak Watermark / Min Free Ever
            tft.setTextColor(bruceConfig.secColor, bruceConfig.bgColor);
            tft.fillRect(marginX, y, barW, LH * FP, bruceConfig.bgColor);
            String peakStr = "Min Free: " + formatBytes(minFreeHeap) + "  Peak: " + formatBytes(peakUsed);
            tft.drawString(peakStr, marginX, y);
            y += LH * FP + 3;

            // 4. PSRAM / DMA Status
            tft.fillRect(marginX, y, barW, LH * FP, bruceConfig.bgColor);
            if (psramFound()) {
                size_t totalPsram = ESP.getPsramSize();
                size_t freePsram = ESP.getFreePsram();
                size_t maxPsram = ESP.getMaxAllocPsram();
                tft.setTextColor(TFT_GREEN, bruceConfig.bgColor);
                String psramStr = "PSRAM: " + formatBytes(freePsram) + " / " + formatBytes(totalPsram) + " (Max: " + formatBytes(maxPsram) + ")";
                tft.drawString(psramStr, marginX, y);
            } else {
                tft.setTextColor(TFT_LIGHTGREY, bruceConfig.bgColor);
                String dmaStr = "PSRAM: None  DMA Free: " + formatBytes(dmaFree);
                tft.drawString(dmaStr, marginX, y);
            }
            y += LH * FP + 3;

            // 5. Integrity Check Status
            tft.fillRect(marginX, y, barW, LH * FP, bruceConfig.bgColor);
            tft.setTextColor(statusColor, bruceConfig.bgColor);
            tft.drawString(integrityStatus, marginX, y);
        }

        vTaskDelay(pdMS_TO_TICKS(15));
    }
}
