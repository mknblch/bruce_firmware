#if !defined(LITE_VERSION)
#include "LoRaRF.h"
#include "LoRaConfig.h"
#include "LoRaRadio.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/utils.h"
#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <vector>

static std::vector<String> chatMessages;
static int chatScrollOffset = 0;
static const int maxChatMessages = 19;
static bool chatNeedUpdate = false;
static String chatOutgoingMsg = "";
static uint8_t chatRxBuffer[256];

static void renderChat() {
    if (!chatNeedUpdate) return;
    tft.setTextSize(FP);
    tft.fillScreen(bruceConfig.bgColor);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);

    String topHeader = "LoRa Chat (" + String(loraConfig.freqMHz, 3) + "M) [" + loraConfig.username + "]";
    tft.drawString(topHeader, 10, 10);

    int yStart = 25;
    int ySpacing = LH * FP + 2;
    int yPos = yStart;

    int endLine = chatScrollOffset + maxChatMessages;
    if (endLine > (int)chatMessages.size()) endLine = chatMessages.size();

    for (int i = chatScrollOffset; i < endLine; i++) {
        tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
        tft.drawString(chatMessages[i], 10, yPos);
        yPos += ySpacing;
    }
    chatNeedUpdate = false;
}

static void loadChatMessages() {
    chatMessages.clear();
    if (!LittleFS.exists("/chats.txt")) {
        File file = LittleFS.open("/chats.txt", "w");
        file.close();
    }
    File file = LittleFS.open("/chats.txt", "r");
    if (file) {
        while (file.available()) {
            String line = file.readStringUntil('\n');
            line.trim();
            if (line.length() > 0) {
                chatMessages.push_back(line);
            }
        }
        file.close();
    }
    if ((int)chatMessages.size() > maxChatMessages) {
        chatScrollOffset = chatMessages.size() - maxChatMessages;
    } else {
        chatScrollOffset = 0;
    }
}

static void sendChatMessage() {
    tft.fillScreen(bruceConfig.bgColor);
    chatOutgoingMsg = keyboard("", 128, "Message:");
    if (chatOutgoingMsg == "" || chatOutgoingMsg == "\x1B") {
        chatNeedUpdate = true;
        return;
    }

    String fullMsg = loraConfig.username + ": " + chatOutgoingMsg;
    if (!transmitLoRaString(fullMsg)) {
        displayError("Send failed");
    } else {
        File file = LittleFS.open("/chats.txt", "a");
        if (file) {
            file.println(fullMsg);
            file.close();
        }
        chatMessages.push_back(fullMsg);
        if ((int)chatMessages.size() > maxChatMessages) {
            chatScrollOffset = chatMessages.size() - maxChatMessages;
        }
    }
    chatOutgoingMsg = "";
    chatNeedUpdate = true;
}

static void receiveChatMessage() {
    float rssi = 0, snr = 0, freqErr = 0;
    size_t pktLen = 0;
    int state = readLoRaRawData(chatRxBuffer, sizeof(chatRxBuffer) - 1, rssi, snr, freqErr, pktLen);
    if (state == RADIOLIB_ERR_NONE && pktLen > 0) {
        chatRxBuffer[pktLen] = '\0';
        String incoming = String((char *)chatRxBuffer);
        incoming.trim();
        if (incoming.length() > 0) {
            File file = LittleFS.open("/chats.txt", "a");
            if (file) {
                file.println(incoming);
                file.close();
            }
            chatMessages.push_back(incoming);
            if ((int)chatMessages.size() > maxChatMessages) {
                chatScrollOffset = chatMessages.size() - maxChatMessages;
            }
            chatNeedUpdate = true;
        }
    }
}

void lorachat() {
    loadLoRaConfig();
    if (!isLoraHardwareConfigured()) {
        displayError("LoRa pins not configured!", true);
        return;
    }

    displayTextLine("Connecting LoRa Chat...");
    if (!initLoRaRadio(loraConfig, true)) {
        displayError("LoRa Init Failed", true);
        return;
    }

    loadChatMessages();
    chatNeedUpdate = true;

    while (true) {
        renderChat();
        receiveChatMessage();

        if (check(EscPress)) break;

        if (check(NextPress) || check(DownPress)) {
            if (chatScrollOffset < (int)chatMessages.size() - maxChatMessages) {
                chatScrollOffset++;
                chatNeedUpdate = true;
            }
        }

        if (check(PrevPress) || check(UpPress)) {
            if (chatScrollOffset > 0) {
                chatScrollOffset--;
                chatNeedUpdate = true;
            }
        }

        if (check(SelPress)) {
            sendChatMessage();
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    stopLoRaRadio();
}

void changeusername() {
    changeLoRaUsername();
}

void chfreq() {
    changeLoRaFrequency();
}

#endif // !LITE_VERSION
