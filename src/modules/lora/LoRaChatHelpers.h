#ifndef __LORA_CHAT_HELPERS_H__
#define __LORA_CHAT_HELPERS_H__

#include "LoRaPacket.h"

namespace LoRaChatHelpers {

inline String decodedDisplayText(const LoRaPacket &packet) {
    if (packet.payloadAscii.isEmpty()) return "";
    if (packet.protocol == LoRaProtocol::BRUCE_CHAT && packet.appName == "CHAT" && !packet.sender.isEmpty()) {
        return packet.sender + ": " + packet.payloadAscii;
    }
    if (packet.protocol == LoRaProtocol::BRUCE_CHAT && packet.appName == "TEXT") return packet.payloadAscii;
    if (packet.protocol == LoRaProtocol::MESHTASTIC && packet.appName == "TEXT") {
        return packet.sender + ": " + packet.payloadAscii;
    }
    return "";
}

} // namespace LoRaChatHelpers

#endif