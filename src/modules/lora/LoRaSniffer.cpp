#if !defined(LITE_VERSION)
#include "LoRaSniffer.h"
#include "LoRaConfig.h"
#include "LoRaPacket.h"
#include "LoRaPcap.h"
#include "LoRaRadio.h"
#include "LoRaTracker.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/utils.h"
#include <Arduino.h>

std::vector<LoRaPacket> gLoRaCapturedPackets;
std::vector<LoRaNodeRecord> gLoRaNodes;
static const size_t MAX_GLOBAL_PACKETS = 100;
static const size_t MAX_NODE_PACKETS = 8;

void addPacketToSniffer(const LoRaPacket &pkt) {
    // Determine address/node key
    String address = "";
    if (pkt.sender.length() > 0) {
        address = pkt.sender;
    } else if (pkt.protocolName == "Meshtastic" && pkt.packetId > 0) {
        char buf[20];
        snprintf(buf, sizeof(buf), "!%08X", (unsigned int)pkt.packetId);
        address = String(buf);
    } else if (pkt.protocolName == "LoRaWAN" && pkt.destination.length() > 0) {
        address = pkt.destination;
    } else if (pkt.protocolName == "Bruce" && pkt.sender.length() > 0) {
        address = pkt.sender;
    } else {
        address = "RAW_" + String(pkt.freqMHz, 2) + "M";
    }

    String displayName = address;
    if (pkt.protocolName == "Meshtastic" && pkt.payloadAscii.length() > 0) {
        displayName = address + " (" + pkt.appName + ")";
    }

    bool found = false;
    for (size_t i = 0; i < gLoRaNodes.size(); i++) {
        if (gLoRaNodes[i].address == address) {
            auto &node = gLoRaNodes[i];
            node.packetCount++;
            node.lastRssi = pkt.rssi;
            if (pkt.rssi > node.peakRssi) node.peakRssi = pkt.rssi;
            node.lastSnr = pkt.snr;
            node.lastSeenMs = millis();
            node.protocol = pkt.protocolName;
            if (displayName.length() > node.displayName.length()) {
                node.displayName = displayName;
            }

            if (node.packets.size() >= MAX_NODE_PACKETS) {
                node.packets.erase(node.packets.begin());
            }
            node.packets.push_back(pkt);
            found = true;
            break;
        }
    }

    if (!found) {
        LoRaNodeRecord newNode;
        newNode.address = address;
        newNode.displayName = displayName;
        newNode.protocol = pkt.protocolName;
        newNode.lastRssi = pkt.rssi;
        newNode.peakRssi = pkt.rssi;
        newNode.lastSnr = pkt.snr;
        newNode.packetCount = 1;
        newNode.lastSeenMs = millis();
        newNode.packets.push_back(pkt);
        gLoRaNodes.push_back(newNode);
    }

    if (gLoRaCapturedPackets.size() >= MAX_GLOBAL_PACKETS) {
        gLoRaCapturedPackets.erase(gLoRaCapturedPackets.begin());
    }
    gLoRaCapturedPackets.push_back(pkt);
}

void showLoRaPacketInspector(const LoRaPacket &pkt) {
    int scroll = 0;
    std::vector<String> lines;

    lines.push_back("=== PHY METADATA ===");
    lines.push_back("Freq: " + String(pkt.freqMHz, 3) + " MHz | SF: " + String(pkt.sf));
    lines.push_back("BW: " + String(pkt.bwKHz, 1) + " kHz | CR: 4/" + String(pkt.cr));
    lines.push_back("Sync: 0x" + String(pkt.syncWord, HEX) + " | CRC: " + (pkt.crcOk ? "OK" : "FAIL"));
    lines.push_back("RSSI: " + String(pkt.rssi, 1) + " dBm | SNR: " + String(pkt.snr, 1) + " dB");
    lines.push_back("FreqErr: " + String(pkt.freqErrorHz, 0) + " Hz");
    lines.push_back("AirTime: " + String(pkt.timeOnAirMs, 1) + " ms | Len: " + String(pkt.raw.size()) + "B");
    lines.push_back("");

    lines.push_back("=== PROTOCOL INFO ===");
    lines.push_back("Proto: " + pkt.protocolName + " (" + pkt.appName + ")");
    if (pkt.sender.length() > 0) lines.push_back("From:  " + pkt.sender);
    if (pkt.destination.length() > 0) lines.push_back("To:    " + pkt.destination);
    if (pkt.packetId > 0) lines.push_back("ID:    0x" + String(pkt.packetId, HEX));
    if (pkt.frameCount > 0) lines.push_back("FCnt:  " + String(pkt.frameCount));
    if (pkt.fPort > 0) lines.push_back("FPort: " + String(pkt.fPort));
    if (pkt.hopLimit >= 0) {
        lines.push_back("Hop:   " + String(pkt.hopLimit) + " / " + String(pkt.hopStart));
    }
    if (pkt.payloadAscii.length() > 0) {
        lines.push_back("Text:  " + pkt.payloadAscii);
    }
    lines.push_back("");

    lines.push_back("=== RAW HEX DUMP ===");
    char hexLine[32];
    for (size_t i = 0; i < pkt.raw.size(); i += 8) {
        String h = "";
        String a = "";
        for (size_t j = 0; j < 8; j++) {
            if (i + j < pkt.raw.size()) {
                snprintf(hexLine, sizeof(hexLine), "%02X ", pkt.raw[i + j]);
                h += hexLine;
                char c = (char)pkt.raw[i + j];
                a += (c >= 32 && c <= 126) ? c : '.';
            } else {
                h += "   ";
            }
        }
        lines.push_back(h + " " + a);
    }

    bool loop = true;
    bool needsRedraw = true;

    // Drain any leftover Enter/Esc press from previous menu
    vTaskDelay(pdMS_TO_TICKS(150));
    check(SelPress);
    check(EscPress);

    while (loop) {
        if (needsRedraw) {
            tft.fillScreen(bruceConfig.bgColor);
            drawMainBorder(true);

            int lineH = FP * LH + 1;
            int maxLines = (tftHeight - BORDER_PAD_Y - 24) / lineH;
            if (maxLines < 1) maxLines = 1;

            tft.setTextSize(FP);
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString("Packet Inspector (" + String(pkt.raw.size()) + "B)", 10, BORDER_PAD_Y);

            int startY = BORDER_PAD_Y + lineH + 2;
            for (int i = 0; i < maxLines; i++) {
                int idx = scroll + i;
                if (idx < (int)lines.size()) {
                    tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
                    if (lines[idx].startsWith("===")) {
                        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
                    }
                    tft.drawString(lines[idx], 10, startY + i * lineH);
                }
            }

        printCenterFootnote("[UP/DN]Scroll [T]Track [ESC]Back");
        needsRedraw = false;
    }

#if defined(HAS_ENCODER)
    int encSteps = (int)drainRotarySteps();
    if (encSteps > 0) {
        int lineH = FP * LH + 1;
        int maxLines = (tftHeight - BORDER_PAD_Y - 24) / lineH;
        if (scroll + maxLines < (int)lines.size()) {
            scroll = min((int)lines.size() - maxLines, scroll + encSteps);
            if (scroll < 0) scroll = 0;
            needsRedraw = true;
        }
    } else if (encSteps < 0) {
        if (scroll > 0) {
            scroll = max(0, scroll + encSteps);
            needsRedraw = true;
        }
    }
#endif

    if (check(NextPress) || check(DownPress) || check(NextPagePress)) {
        int lineH = FP * LH + 1;
        int maxLines = (tftHeight - BORDER_PAD_Y - 24) / lineH;
        if (scroll + maxLines < (int)lines.size()) {
            scroll++;
            needsRedraw = true;
        }
    }
    if (check(PrevPress) || check(UpPress) || check(PrevPagePress)) {
        if (scroll > 0) {
            scroll--;
            needsRedraw = true;
        }
    }
    if (check(EscPress) || check(SelPress)) {
        loop = false;
        break;
    }
    keyStroke k = _getKeyPress();
    if (k.pressed || !k.word.empty()) {
        if (k.del || k.exit_key) {
            loop = false;
            break;
        }
        for (auto ch : k.word) {
            char lowerKey = tolower(ch);
            if (lowerKey == '`') {
                loop = false;
                break;
            } else if (lowerKey == 't') {
                if (pkt.sender.length() > 0) {
                    trackLoRaTarget(pkt.sender, pkt.sender);
                    loop = false;
                    break;
                }
            }
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}
}

void showLoRaNodeInspector(LoRaNodeRecord &node) {
    int selectedPktIdx = 0;
    int scrollOffset = 0;
    bool loop = true;
    bool needsRedraw = true;

    // Drain any leftover Enter/Esc press from previous menu
    vTaskDelay(pdMS_TO_TICKS(150));
    check(SelPress);
    check(EscPress);

    drawMainBorder(true);

    while (loop) {
        bool up = check(PrevPress) || check(UpPress) || check(PrevPagePress);
        bool down = check(NextPress) || check(DownPress) || check(NextPagePress);
        bool sel = check(SelPress);
        bool esc = check(EscPress);

#if defined(HAS_ENCODER)
        int encSteps = (int)drainRotarySteps();
        if (encSteps > 0) down = true;
        else if (encSteps < 0) up = true;
#endif

        if (esc) {
            break;
        }

        if (up) {
            if (selectedPktIdx > 0) {
                selectedPktIdx--;
                needsRedraw = true;
            }
        }
        if (down) {
            if (selectedPktIdx + 1 < (int)node.packets.size()) {
                selectedPktIdx++;
                needsRedraw = true;
            }
        }

        keyStroke k = _getKeyPress();
        if (k.pressed || !k.word.empty()) {
            if (k.del || k.exit_key) {
                break;
            }
            for (auto ch : k.word) {
                char lowerKey = tolower(ch);
                if (lowerKey == '`') {
                    loop = false;
                    break;
                } else if (lowerKey == 't') {
                    trackLoRaTarget(node.address, node.displayName);
                    drawMainBorder(true);
                    needsRedraw = true;
                } else if (lowerKey == 'e') {
                    String path = "";
                    if (exportLoRaPacketsToPcap(node.packets, path)) {
                        displaySuccess("PCAP: " + path);
                        drawMainBorder(true);
                        needsRedraw = true;
                    }
                }
            }
        }
        if (!loop) break;

        if (sel) {
            if (!node.packets.empty() && selectedPktIdx >= 0 && selectedPktIdx < (int)node.packets.size()) {
                // Show newest first in index mapping
                size_t actualIdx = node.packets.size() - 1 - selectedPktIdx;
                showLoRaPacketInspector(node.packets[actualIdx]);
                drawMainBorder(true);
                needsRedraw = true;
            }
        }

        if (needsRedraw) {
            needsRedraw = false;
            tft.fillScreen(bruceConfig.bgColor);
            drawMainBorder(true);

            int lineH = FP * LH + 1;
            tft.setTextSize(FP);

            // Node summary header
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString("MAC/Node: " + node.address, 10, BORDER_PAD_Y);

            tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
            String sub = "Proto: " + node.protocol + " | Total: " + String(node.packetCount);
            tft.drawString(sub, 10, BORDER_PAD_Y + lineH);

            String sig = "RSSI: " + String(node.lastRssi, 1) + "dBm | SNR: " + String(node.lastSnr, 1) + "dB";
            tft.drawString(sig, 10, BORDER_PAD_Y + 2 * lineH);

            int startY = BORDER_PAD_Y + 3 * lineH + 3;
            int maxLines = (tftHeight - startY - 14) / lineH;
            if (maxLines < 1) maxLines = 1;

            if (node.packets.empty()) {
                tft.setTextColor(TFT_DARKGREY, bruceConfig.bgColor);
                tft.drawString("No stored packets", 10, startY);
            } else {
                if (selectedPktIdx < scrollOffset) scrollOffset = selectedPktIdx;
                if (selectedPktIdx >= scrollOffset + maxLines) {
                    scrollOffset = selectedPktIdx - maxLines + 1;
                }

                for (int i = 0; i < maxLines; i++) {
                    int pIdx = scrollOffset + i;
                    int currentY = startY + i * lineH;
                    if (pIdx < (int)node.packets.size()) {
                        size_t actualIdx = node.packets.size() - 1 - pIdx;
                        const auto &pkt = node.packets[actualIdx];
                        bool isSel = (pIdx == selectedPktIdx);

                        if (isSel) {
                            tft.fillRect(7, currentY, tftWidth - 14, lineH, bruceConfig.priColor);
                            tft.setTextColor(bruceConfig.bgColor, bruceConfig.priColor);
                        } else {
                            tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
                        }

                        String line = "#" + String(pIdx + 1) + " [" + String((int)pkt.rssi) + "dBm] " + String(pkt.raw.size()) + "B";
                        if (pkt.payloadAscii.length() > 0) {
                            line += " " + pkt.payloadAscii;
                        } else if (pkt.appName.length() > 0) {
                            line += " (" + pkt.appName + ")";
                        }
                        if (line.length() > 30) line = line.substring(0, 30);
                        tft.drawString(line, 10, currentY);
                    }
                }
            }

            printCenterFootnote("[SEL]Inspect [T]Track [ESC]Back");
        }

        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

void viewLoRaCapturedPackets() {
    if (gLoRaCapturedPackets.empty()) {
        displayError("No captured packets", true);
        return;
    }

    std::vector<Option> options;
    for (size_t i = 0; i < gLoRaCapturedPackets.size(); i++) {
        size_t idx = gLoRaCapturedPackets.size() - 1 - i;
        const auto &pkt = gLoRaCapturedPackets[idx];
        String title = String(idx + 1) + ". " + pkt.summary + " (" + String((int)pkt.rssi) + "dBm)";
        options.push_back({title, [idx]() {
            showLoRaPacketInspector(gLoRaCapturedPackets[idx]);
        }});
    }

    options.push_back({"Export all to PCAP", []() {
        String path = "";
        if (exportLoRaPacketsToPcap(gLoRaCapturedPackets, path)) {
            displaySuccess("Saved to " + path);
        } else {
            displayError("Export failed");
        }
    }});

    options.push_back({"Clear buffer", []() {
        gLoRaCapturedPackets.clear();
        gLoRaNodes.clear();
        displaySuccess("Cleared");
    }});

    loopOptions(options, MENU_TYPE_SUBMENU, "Captured Packets");
}

void runLoRaSniffer() {
    loadLoRaConfig();
    if (!isLoraHardwareConfigured()) {
        displayError("LoRa pins not configured!", true);
        return;
    }

    displayTextLine("Starting LoRa Sniffer...");
    if (!initLoRaRadio(loraConfig, true)) {
        displayError("LoRa Radio Init Failed", true);
        return;
    }

    LoRaPcapWriter pcap;
    bool pcapActive = false;
    if (loraConfig.enablePcap) {
        pcapActive = pcap.begin();
    }

    bool isPaused = false;
    int selectedIdx = 0;
    int scrollOffset = 0;
    uint32_t totalPackets = gLoRaCapturedPackets.size();
    uint32_t lastUiUpdate = 0;
    bool needsRedraw = true;
    bool statsChanged = false;

    drawMainBorder(true);
    uint8_t rxBuffer[256];

    // Drain any leftover Enter/Esc press from previous menu
    vTaskDelay(pdMS_TO_TICKS(150));
    check(SelPress);
    check(EscPress);

    while (true) {
        bool up = check(PrevPress) || check(UpPress) || check(PrevPagePress);
        bool down = check(NextPress) || check(DownPress) || check(NextPagePress);
        bool sel = check(SelPress);
        bool esc = check(EscPress);

#if defined(HAS_ENCODER)
        int encSteps = (int)drainRotarySteps();
        if (encSteps > 0) down = true;
        else if (encSteps < 0) up = true;
#endif

        if (esc) {
            break;
        }

        if (up) {
            if (selectedIdx > 0) {
                selectedIdx--;
                needsRedraw = true;
            }
        }
        if (down) {
            if (selectedIdx + 1 < (int)gLoRaNodes.size()) {
                selectedIdx++;
                needsRedraw = true;
            }
        }

        keyStroke k = _getKeyPress();
        if (k.pressed || !k.word.empty()) {
            if (k.del || k.exit_key) {
                break;
            }
            for (auto ch : k.word) {
                char lowerKey = tolower(ch);
                if (lowerKey == '`') {
                    goto exit_sniffer;
                } else if (lowerKey == 'p') {
                    isPaused = !isPaused;
                    needsRedraw = true;
                } else if (lowerKey == 'c') {
                    gLoRaCapturedPackets.clear();
                    gLoRaNodes.clear();
                    selectedIdx = 0;
                    scrollOffset = 0;
                    totalPackets = 0;
                    needsRedraw = true;
                } else if (lowerKey == 'e') {
                    String path = "";
                    if (exportLoRaPacketsToPcap(gLoRaCapturedPackets, path)) {
                        displaySuccess("PCAP: " + path);
                        drawMainBorder(true);
                        needsRedraw = true;
                    }
                } else if (lowerKey == 't' && !gLoRaNodes.empty()) {
                    if (selectedIdx >= 0 && selectedIdx < (int)gLoRaNodes.size()) {
                        pcap.end();
                        stopLoRaRadio();
                        trackLoRaTarget(gLoRaNodes[selectedIdx].address, gLoRaNodes[selectedIdx].displayName);
                        return;
                    }
                }
            }
        }

        if (sel) {
            if (!gLoRaNodes.empty() && selectedIdx >= 0 && selectedIdx < (int)gLoRaNodes.size()) {
                showLoRaNodeInspector(gLoRaNodes[selectedIdx]);
                drawMainBorder(true);
                needsRedraw = true;
            } else {
                // If empty list, toggle pause/action
                isPaused = !isPaused;
                needsRedraw = true;
            }
        }

        // Check for incoming packet
        if (!isPaused && checkLoRaPacketAvailable()) {
            float rssi = 0, snr = 0, freqErr = 0;
            size_t pktLen = 0;
            int state = readLoRaRawData(rxBuffer, sizeof(rxBuffer), rssi, snr, freqErr, pktLen);

            if (state == RADIOLIB_ERR_NONE && pktLen > 0) {
                LoRaPacket pkt;
                pkt.timestampMs = millis();
                pkt.freqMHz = loraConfig.freqMHz;
                pkt.sf = loraConfig.sf;
                pkt.bwKHz = loraConfig.bwKHz;
                pkt.cr = loraConfig.cr;
                pkt.syncWord = loraConfig.syncWord;
                pkt.rssi = rssi;
                pkt.snr = snr;
                pkt.freqErrorHz = freqErr;
                pkt.timeOnAirMs = getLoRaTimeOnAir(pktLen);
                pkt.crcOk = true;
                pkt.raw.assign(rxBuffer, rxBuffer + pktLen);

                parseLoRaPacket(pkt);
                addPacketToSniffer(pkt);
                totalPackets++;

                if (pcapActive) {
                    pcap.writePacket(pkt);
                }

                statsChanged = true;
            }
        }

        // Render UI
        if (needsRedraw || ((millis() - lastUiUpdate > 250) && statsChanged)) {
            lastUiUpdate = millis();
            needsRedraw = false;
            statsChanged = false;

            int lineH = FP * LH + 1;
            int maxLines = (tftHeight - BORDER_PAD_Y - 24) / lineH;
            if (maxLines < 1) maxLines = 1;
            int startY = BORDER_PAD_Y + lineH;

            // Status bar
            tft.setTextSize(FP);
            tft.fillRect(7, BORDER_PAD_Y, tftWidth - 14, lineH, bruceConfig.bgColor);

            String status = "LoRa: " + String(gLoRaNodes.size()) + " Nodes (" + String(totalPackets) + " Pkts)";
            if (pcapActive) status += " [PCAP]";
            if (isPaused) status += " [PAUSE]";
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.drawString(status, 10, BORDER_PAD_Y);

            if (gLoRaNodes.empty()) {
                tft.fillRect(7, startY, tftWidth - 14, (tftHeight - startY - 14), bruceConfig.bgColor);
                tft.setTextColor(TFT_DARKGREY, bruceConfig.bgColor);
                tft.drawString("Listening on " + String(loraConfig.freqMHz, 3) + " MHz (SF" + String(loraConfig.sf) + ")", 10, startY + 4);
                tft.drawString("No packets captured yet", 10, startY + lineH + 6);
                printCenterFootnote("[ESC]Exit [P]Pause [C]Clear");
            } else {
                if (selectedIdx < scrollOffset) scrollOffset = selectedIdx;
                if (selectedIdx >= scrollOffset + maxLines) {
                    scrollOffset = selectedIdx - maxLines + 1;
                }

                for (int i = 0; i < maxLines; i++) {
                    int nIdx = scrollOffset + i;
                    int currentY = startY + i * lineH;
                    tft.fillRect(7, currentY, tftWidth - 14, lineH, bruceConfig.bgColor);

                    if (nIdx < (int)gLoRaNodes.size()) {
                        const auto &node = gLoRaNodes[nIdx];
                        bool isSel = (nIdx == selectedIdx);

                        if (isSel) {
                            tft.fillRect(7, currentY, tftWidth - 14, lineH, bruceConfig.priColor);
                            tft.setTextColor(bruceConfig.bgColor, bruceConfig.priColor);
                        } else {
                            tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
                        }

                        // Display formatted row: "!1a2b3c4d [Mesh] -85dBm (5pkts)"
                        String line = node.address + " [" + node.protocol.substring(0, 4) + "] " + String((int)node.lastRssi) + "dBm (" + String(node.packetCount) + ")";
                        if (line.length() > 32) line = line.substring(0, 32);
                        tft.drawString(line, 10, currentY);
                    }
                }
                printCenterFootnote("[UP/DN]Select [SEL]Inspect [ESC]Back");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

exit_sniffer:
    if (pcapActive) {
        pcap.end();
    }
    stopLoRaRadio();
}

#endif // !LITE_VERSION
