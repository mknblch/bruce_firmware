#if !defined(LITE_VERSION)
#include "LoRaRF.h"
#include "LoRaChatHelpers.h"
#include "LoRaChatStorageHelpers.h"
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
static void persistChatMessages();

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
        LoRaChatStorageHelpers::loadMessages(file, chatMessages);
        file.close();
    }
    chatScrollOffset = (chatMessages.size() > maxChatMessages) ? chatMessages.size() - maxChatMessages : 0;
    persistChatMessages();
}

static String normalizeChatLine(const String &message) {
    String normalized = "";
    for (size_t i = 0; i < message.length(); i++) {
        const unsigned char character = (unsigned char)message[i];
        if (character >= 32 && character <= 126) {
            normalized += (char)character;
        } else if (character == '\t' || character == '\r' || character == '\n') {
            normalized += ' ';
        }
    }
    normalized.trim();
    if (normalized.length() > LoRaChatStorageHelpers::MAX_CHAT_MESSAGE_LENGTH) {
        normalized = normalized.substring(0, LoRaChatStorageHelpers::MAX_CHAT_MESSAGE_LENGTH);
    }
    return normalized;
}

static bool appendChatMessage(const String &message) {
    const String normalized = normalizeChatLine(message);
    if (normalized.isEmpty()) return false;
    LoRaChatStorageHelpers::appendBoundedMessage(chatMessages, normalized);
    chatScrollOffset = (chatMessages.size() > maxChatMessages) ? chatMessages.size() - maxChatMessages : 0;
    chatNeedUpdate = true;
    return true;
}

static void persistChatMessages() {
    while (LoRaChatStorageHelpers::serializedSize(chatMessages) > LoRaChatStorageHelpers::MAX_CHAT_FILE_BYTES &&
           !chatMessages.empty()) {
        chatMessages.erase(chatMessages.begin());
    }
    File file = LittleFS.open("/chats.txt", "w");
    if (!file) return;
    for (const String &message : chatMessages) file.println(message);
    file.close();
}

static void sendChatMessage() {
    tft.fillScreen(bruceConfig.bgColor);
    chatOutgoingMsg = keyboard("", 128, "Message:");
    if (chatOutgoingMsg == "" || chatOutgoingMsg == "\x1B") {
        chatNeedUpdate = true;
        return;
    }

    String fullMsg = normalizeChatLine(loraConfig.username + ": " + chatOutgoingMsg);
    if (fullMsg.isEmpty()) {
        chatNeedUpdate = true;
        return;
    }
    if (!transmitLoRaString(fullMsg)) {
        displayError("Send failed");
    } else {
        appendChatMessage(fullMsg);
        persistChatMessages();
    }
    chatOutgoingMsg = "";
    chatNeedUpdate = true;
}

static void receiveChatMessage() {
    float rssi = 0, snr = 0, freqErr = 0;
    size_t pktLen = 0;
    const int state = readLoRaRawData(chatRxBuffer, sizeof(chatRxBuffer), rssi, snr, freqErr, pktLen);
    if (state == RADIOLIB_ERR_NONE && pktLen > 0 && pktLen <= sizeof(chatRxBuffer)) {
        LoRaPacket packet;
        packet.timestampMs = millis();
        packet.freqMHz = loraConfig.freqMHz;
        packet.sf = loraConfig.sf;
        packet.bwKHz = loraConfig.bwKHz;
        packet.cr = loraConfig.cr;
        packet.syncWord = loraConfig.syncWord;
        packet.rssi = rssi;
        packet.snr = snr;
        packet.freqErrorHz = freqErr;
        packet.timeOnAirMs = getLoRaTimeOnAir(pktLen);
        packet.crcOk = true;
        packet.raw.assign(chatRxBuffer, chatRxBuffer + pktLen);
        parseLoRaPacket(packet);

        const String incoming = LoRaChatHelpers::decodedDisplayText(packet);
        if (appendChatMessage(incoming)) persistChatMessages();
    }
}

static int maxChatScrollOffset() {
    return (chatMessages.size() > maxChatMessages) ? chatMessages.size() - maxChatMessages : 0;
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

#if defined(HAS_ENCODER)
        int32_t encSteps = drainRotarySteps();
        if (encSteps != 0) {
            check(PrevPress);
            check(NextPress);
            check(UpPress);
            check(DownPress);
            check(PrevPagePress);
            check(NextPagePress);
            while (encSteps > 0) {
                if (chatScrollOffset > 0) {
                    chatScrollOffset--;
                    chatNeedUpdate = true;
                }
                encSteps--;
            }
            while (encSteps < 0) {
                if (chatScrollOffset < maxChatScrollOffset()) {
                    chatScrollOffset++;
                    chatNeedUpdate = true;
                }
                encSteps++;
            }
            PrevPress = false;
            NextPress = false;
            UpPress = false;
            DownPress = false;
            PrevPagePress = false;
            NextPagePress = false;
        } else
#endif
        {
            if (check(NextPress) || check(DownPress)) {
                if (chatScrollOffset < maxChatScrollOffset()) {
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
