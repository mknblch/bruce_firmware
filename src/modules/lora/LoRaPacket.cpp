#if !defined(LITE_VERSION)
#include "LoRaPacket.h"

static const char *getMeshtasticPortName(uint32_t portNum) {
    switch (portNum) {
        case 1:  return "TEXT";
        case 2:  return "REMOTE_HW";
        case 3:  return "POSITION";
        case 4:  return "NODEINFO";
        case 5:  return "ROUTING";
        case 6:  return "ADMIN";
        case 7:  return "TEXT_COMP";
        case 8:  return "WAYPOINT";
        case 9:  return "AUDIO";
        case 10: return "DETECTION";
        case 32: return "TELEMETRY";
        case 33: return "ZPS";
        case 34: return "SIMULATOR";
        case 35: return "TRACEROUTE";
        case 67: return "NEIGHBOR";
        case 70: return "MAP_REPORT";
        case 72: return "PAXCOUNTER";
        default: return "DATA";
    }
}

static const char *getLoRaWANMTypeName(uint8_t mType) {
    switch (mType) {
        case 0: return "Join-Req";
        case 1: return "Join-Accept";
        case 2: return "Unconf Up";
        case 3: return "Unconf Down";
        case 4: return "Conf Up";
        case 5: return "Conf Down";
        case 6: return "Rejoin-Req";
        case 7: return "Proprietary";
        default: return "Unknown";
    }
}

String formatMacAddress(const uint8_t *mac, size_t len) {
    String out = "";
    char buf[4];
    for (size_t i = 0; i < len; i++) {
        snprintf(buf, sizeof(buf), "%02X", mac[i]);
        out += buf;
        if (i + 1 < len) out += ":";
    }
    return out;
}

String formatHexDump(const uint8_t *data, size_t len, size_t maxBytes) {
    String out = "";
    size_t count = (len > maxBytes) ? maxBytes : len;
    char hexBuf[4];
    for (size_t i = 0; i < count; i++) {
        snprintf(hexBuf, sizeof(hexBuf), "%02X ", data[i]);
        out += hexBuf;
        if ((i + 1) % 16 == 0 && (i + 1 < count)) {
            out += "\n";
        }
    }
    if (len > maxBytes) {
        out += "\n... (" + String(len) + " bytes total)";
    }
    return out;
}

static bool isPrintableAsciiString(const uint8_t *data, size_t len) {
    if (len == 0) return false;
    size_t printable = 0;
    for (size_t i = 0; i < len; i++) {
        if (data[i] >= 32 && data[i] <= 126) printable++;
        else if (data[i] == '\r' || data[i] == '\n' || data[i] == '\t') printable++;
    }
    return ((float)printable / (float)len) >= 0.85f;
}

static String extractAscii(const uint8_t *data, size_t len) {
    String s = "";
    for (size_t i = 0; i < len; i++) {
        if (data[i] >= 32 && data[i] <= 126) {
            s += (char)data[i];
        } else {
            s += '.';
        }
    }
    return s;
}

static bool parseMeshtastic(LoRaPacket &pkt) {
    const size_t len = pkt.raw.size();
    if (len < 16) return false;

    // Meshtastic Header: 16 bytes
    // [0..3]: to (uint32_t LE)
    // [4..7]: from (uint32_t LE)
    // [8..11]: id (uint32_t LE)
    // [12]: flags (hop_limit: bits 0..2, want_ack: bit 3, via_mqtt: bit 4, hop_start: bits 5..7)
    // [13]: channel_hash
    // [14..15]: reserved / rx_time
    const uint8_t *p = pkt.raw.data();

    uint32_t to = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    uint32_t from = (uint32_t)p[4] | ((uint32_t)p[5] << 8) | ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24);
    uint32_t id = (uint32_t)p[8] | ((uint32_t)p[9] << 8) | ((uint32_t)p[10] << 16) | ((uint32_t)p[11] << 24);
    uint8_t flags = p[12];
    uint8_t chHash = p[13];

    // Sanity heuristic: from node ID shouldn't be all 0x00 or all 0xFF
    if (from == 0x00000000 || from == 0xFFFFFFFF) return false;

    pkt.protocol = LoRaProtocol::MESHTASTIC;
    pkt.protocolName = "Meshtastic";
    pkt.packetId = id;
    pkt.channelHash = chHash;
    pkt.hopLimit = flags & 0x07;
    pkt.wantAck = (flags & 0x08) != 0;
    pkt.viaMqtt = (flags & 0x10) != 0;
    pkt.hopStart = (flags >> 5) & 0x07;

    char buf[20];
    snprintf(buf, sizeof(buf), "!%08x", from);
    pkt.sender = buf;

    if (to == 0xFFFFFFFF) {
        pkt.destination = "^all (Broadcast)";
    } else {
        snprintf(buf, sizeof(buf), "!%08x", to);
        pkt.destination = buf;
    }

    // Try parsing unencrypted Data protobuf payload (tag 1: portnum, tag 2: payload)
    String textMsg = "";
    String appStr = "ENCRYPTED/RAW";
    size_t payloadLen = len - 16;
    const uint8_t *payload = p + 16;

    if (payloadLen > 0) {
        size_t idx = 0;
        uint32_t portNum = 0;
        bool foundPort = false;

        while (idx < payloadLen) {
            uint8_t tag = payload[idx++];
            uint8_t wireType = tag & 0x07;
            uint32_t fieldNum = tag >> 3;

            if (fieldNum == 1 && wireType == 0) { // portnum (varint)
                uint32_t val = 0;
                int shift = 0;
                while (idx < payloadLen) {
                    uint8_t b = payload[idx++];
                    val |= (b & 0x7F) << shift;
                    if (!(b & 0x80)) break;
                    shift += 7;
                }
                portNum = val;
                foundPort = true;
                appStr = getMeshtasticPortName(portNum);
            } else if (fieldNum == 2 && wireType == 2) { // payload (bytes)
                uint32_t strLen = 0;
                int shift = 0;
                while (idx < payloadLen) {
                    uint8_t b = payload[idx++];
                    strLen |= (b & 0x7F) << shift;
                    if (!(b & 0x80)) break;
                    shift += 7;
                }
                if (idx + strLen <= payloadLen) {
                    if (portNum == 1 || portNum == 0) { // TEXT_MESSAGE_APP
                        textMsg = extractAscii(payload + idx, strLen);
                    }
                    idx += strLen;
                }
            } else {
                break;
            }
        }

        if (!foundPort && isPrintableAsciiString(payload, payloadLen)) {
            textMsg = extractAscii(payload, payloadLen);
            appStr = "TEXT_RAW";
        }
    }

    pkt.appName = appStr;
    pkt.payloadAscii = textMsg;

    pkt.summary = "[" + String(appStr) + "] " + pkt.sender + " -> " + (to == 0xFFFFFFFF ? "^all" : pkt.destination);
    if (textMsg.length() > 0) {
        pkt.summary += ": \"" + textMsg.substring(0, 20) + "\"";
    }

    pkt.details = "Proto: Meshtastic (" + appStr + ")\n";
    pkt.details += "From:  " + pkt.sender + "\n";
    pkt.details += "To:    " + pkt.destination + "\n";
    pkt.details += "ID:    0x" + String(id, HEX) + " | ChHash: 0x" + String(chHash, HEX) + "\n";
    pkt.details += "Hop:   " + String(pkt.hopLimit) + " / " + String(pkt.hopStart);
    if (pkt.wantAck) pkt.details += " [WantAck]";
    if (pkt.viaMqtt) pkt.details += " [MQTT]";
    pkt.details += "\n";

    if (textMsg.length() > 0) {
        pkt.details += "Text:  " + textMsg + "\n";
    }

    return true;
}

static bool parseLoRaWAN(LoRaPacket &pkt) {
    const size_t len = pkt.raw.size();
    if (len < 12) return false; // Minimum LoRaWAN Data Frame length with MIC is 12

    const uint8_t *p = pkt.raw.data();
    uint8_t mhdr = p[0];
    uint8_t mType = (mhdr >> 5) & 0x07;
    uint8_t major = mhdr & 0x03;

    if (major != 0) return false; // Only LoRaWAN R1 supported

    pkt.mType = mType;
    pkt.protocol = LoRaProtocol::LORAWAN;
    pkt.protocolName = "LoRaWAN";
    String mTypeName = getLoRaWANMTypeName(mType);
    pkt.appName = mTypeName;

    char hexBuf[24];

    if (mType == 0) { // Join-Request (23 bytes)
        if (len < 23) return false;
        // AppEUI (8B LE) + DevEUI (8B LE) + DevNonce (2B LE) + MIC (4B)
        snprintf(
            hexBuf, sizeof(hexBuf), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
            p[16], p[15], p[14], p[13], p[12], p[11], p[10], p[9]
        );
        pkt.sender = "DevEUI: " + String(hexBuf);

        snprintf(
            hexBuf, sizeof(hexBuf), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
            p[8], p[7], p[6], p[5], p[4], p[3], p[2], p[1]
        );
        pkt.destination = "JoinEUI: " + String(hexBuf);

        uint16_t devNonce = (uint16_t)p[17] | ((uint16_t)p[18] << 8);
        pkt.packetId = devNonce;

        pkt.summary = "[Join-Req] " + pkt.sender;
        pkt.details = "Proto: LoRaWAN (Join-Request)\n";
        pkt.details += pkt.sender + "\n";
        pkt.details += pkt.destination + "\n";
        pkt.details += "Nonce: 0x" + String(devNonce, HEX) + "\n";
        return true;
    } else if (mType >= 2 && mType <= 5) { // Data Frame (Unconfirmed / Confirmed Up/Down)
        // DevAddr (4B LE)
        snprintf(hexBuf, sizeof(hexBuf), "%02X:%02X:%02X:%02X", p[4], p[3], p[2], p[1]);
        pkt.sender = "DevAddr " + String(hexBuf);

        uint8_t fCtrl = p[5];
        uint8_t fOptsLen = fCtrl & 0x0F;
        uint16_t fCnt = (uint16_t)p[6] | ((uint16_t)p[7] << 8);
        pkt.frameCount = fCnt;

        size_t fPortIdx = 8 + fOptsLen;
        if (fPortIdx < len - 4) {
            pkt.fPort = p[fPortIdx];
            size_t payloadStart = fPortIdx + 1;
            size_t payloadLen = (len >= 4 + payloadStart) ? (len - 4 - payloadStart) : 0;
            if (payloadLen > 0 && isPrintableAsciiString(p + payloadStart, payloadLen)) {
                pkt.payloadAscii = extractAscii(p + payloadStart, payloadLen);
            }
        }

        pkt.summary = "[" + mTypeName + "] " + pkt.sender + " FCnt:" + String(fCnt);
        if (pkt.fPort > 0) pkt.summary += " FPort:" + String(pkt.fPort);

        pkt.details = "Proto: LoRaWAN (" + mTypeName + ")\n";
        pkt.details += "DevAddr:  " + String(hexBuf) + "\n";
        pkt.details += "FCnt:     " + String(fCnt) + " | FCtrl: 0x" + String(fCtrl, HEX) + "\n";
        pkt.details += "FPort:    " + String(pkt.fPort) + " | Len: " + String(len) + "B\n";
        if (pkt.payloadAscii.length() > 0) {
            pkt.details += "Payload:  " + pkt.payloadAscii + "\n";
        }
        return true;
    }

    return false;
}

static bool parseBruceChat(LoRaPacket &pkt) {
    const size_t len = pkt.raw.size();
    if (len < 3) return false;

    String ascii = extractAscii(pkt.raw.data(), len);
    int colonPos = ascii.indexOf(':');

    if (colonPos > 0 && colonPos < 24) {
        pkt.protocol = LoRaProtocol::BRUCE_CHAT;
        pkt.protocolName = "Bruce Chat";
        pkt.sender = ascii.substring(0, colonPos);
        pkt.sender.trim();
        pkt.destination = "Broadcast";
        pkt.payloadAscii = ascii.substring(colonPos + 1);
        pkt.payloadAscii.trim();
        pkt.appName = "CHAT";

        pkt.summary = "[Chat] " + pkt.sender + ": " + pkt.payloadAscii;
        pkt.details = "Proto: Bruce LoRa Chat\n";
        pkt.details += "From:  " + pkt.sender + "\n";
        pkt.details += "Msg:   " + pkt.payloadAscii + "\n";
        return true;
    }

    if (isPrintableAsciiString(pkt.raw.data(), len)) {
        pkt.protocol = LoRaProtocol::BRUCE_CHAT;
        pkt.protocolName = "ASCII Text";
        pkt.sender = "Node";
        pkt.destination = "Broadcast";
        pkt.payloadAscii = ascii;
        pkt.appName = "TEXT";

        pkt.summary = "[ASCII] " + ascii.substring(0, 24);
        pkt.details = "Proto: Plain ASCII\n";
        pkt.details += "Data:  " + ascii + "\n";
        return true;
    }

    return false;
}

void parseLoRaPacket(LoRaPacket &pkt) {
    if (pkt.raw.empty()) return;

    // Check Sync Word hints first
    if (pkt.syncWord == 0x2B) { // Meshtastic sync word
        if (parseMeshtastic(pkt)) return;
    } else if (pkt.syncWord == 0x34) { // LoRaWAN sync word
        if (parseLoRaWAN(pkt)) return;
    }

    // Try all parsers heuristically
    if (parseMeshtastic(pkt)) return;
    if (parseLoRaWAN(pkt)) return;
    if (parseBruceChat(pkt)) return;

    // Fallback: Generic RAW packet
    pkt.protocol = LoRaProtocol::RAW;
    pkt.protocolName = "RAW";
    pkt.sender = "0x" + String(pkt.raw[0], HEX);
    pkt.destination = "Broadcast";
    pkt.appName = "RAW";

    String hexHead = "";
    size_t count = (pkt.raw.size() > 8) ? 8 : pkt.raw.size();
    char buf[4];
    for (size_t i = 0; i < count; i++) {
        snprintf(buf, sizeof(buf), "%02X ", pkt.raw[i]);
        hexHead += buf;
    }
    pkt.summary = "[RAW " + String(pkt.raw.size()) + "B] " + hexHead;
    pkt.details = "Proto: RAW LoRa Packet\n";
    pkt.details += "Len:   " + String(pkt.raw.size()) + " bytes\n";
    pkt.details += "Hex:   \n" + formatHexDump(pkt.raw.data(), pkt.raw.size(), 32) + "\n";
}

#endif // !LITE_VERSION
