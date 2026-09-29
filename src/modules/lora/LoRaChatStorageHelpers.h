#ifndef __LORA_CHAT_STORAGE_HELPERS_H__
#define __LORA_CHAT_STORAGE_HELPERS_H__

#include <cstddef>
#include <cstdint>

namespace LoRaChatStorageHelpers {

static const size_t MAX_CHAT_HISTORY_MESSAGES = 64;
static const size_t MAX_CHAT_MESSAGE_LENGTH = 180;
static const size_t MAX_CHAT_FILE_BYTES = MAX_CHAT_HISTORY_MESSAGES * (MAX_CHAT_MESSAGE_LENGTH + 2);

template <typename MessageList>
void retainLastMessages(MessageList &messages, size_t capacity) {
    while (messages.size() > capacity) messages.erase(messages.begin());
}

template <typename MessageList>
void appendBoundedMessage(MessageList &messages, const typename MessageList::value_type &message) {
    typename MessageList::value_type bounded = message;
    if (bounded.length() > MAX_CHAT_MESSAGE_LENGTH) {
        bounded = bounded.substring(0, MAX_CHAT_MESSAGE_LENGTH);
    }
    messages.push_back(bounded);
    retainLastMessages(messages, MAX_CHAT_HISTORY_MESSAGES);
}

template <typename MessageList>
size_t serializedSize(const MessageList &messages) {
    size_t size = 0;
    for (const auto &message : messages) size += message.length() + 2;
    return size;
}

template <typename FileType, typename MessageList>
void loadMessages(FileType &file, MessageList &messages) {
    messages.clear();
    if (file.size() > MAX_CHAT_FILE_BYTES) {
        const uint32_t start = file.size() - MAX_CHAT_FILE_BYTES;
        if (file.seek(start)) {
            while (file.available() && file.read() != '\n') {}
        }
    }

    typename MessageList::value_type line = "";
    while (file.available()) {
        const int character = file.read();
        if (character == '\n') {
            line.trim();
            if (line.length() > 0) appendBoundedMessage(messages, line);
            line = "";
        } else if (character >= 32 && character <= 126 && line.length() < MAX_CHAT_MESSAGE_LENGTH) {
            line += (char)character;
        }
    }
    line.trim();
    if (line.length() > 0) appendBoundedMessage(messages, line);
}

} // namespace LoRaChatStorageHelpers

#endif