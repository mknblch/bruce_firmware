#ifndef __LORA_SNIFFER_STORAGE_H__
#define __LORA_SNIFFER_STORAGE_H__

#include <cstddef>
#include <cstdint>

namespace LoRaSnifferStorage {

static const size_t MAX_NODE_RECORDS = 32;
static const size_t MAX_NODE_PACKET_IDS = 8;

template <typename NodeList>
size_t oldestNodeIndex(const NodeList &nodes, uint32_t nowMs) {
    size_t oldestIndex = 0;
    uint32_t oldestAge = 0;
    for (size_t i = 0; i < nodes.size(); i++) {
        const uint32_t age = nowMs - nodes[i].lastSeenMs;
        if (i == 0 || age > oldestAge) {
            oldestIndex = i;
            oldestAge = age;
        }
    }
    return oldestIndex;
}

template <typename NodeList>
bool evictOldestIfAtCapacity(NodeList &nodes, size_t capacity, uint32_t nowMs) {
    if (capacity == 0 || nodes.size() < capacity) return false;
    nodes.erase(nodes.begin() + oldestNodeIndex(nodes, nowMs));
    return true;
}

template <typename CaptureIdList>
void appendRecentCaptureId(CaptureIdList &captureIds, uint32_t captureId, size_t capacity) {
    if (capacity == 0) return;
    if (captureIds.size() >= capacity) captureIds.erase(captureIds.begin());
    captureIds.push_back(captureId);
}

template <typename CaptureList>
const typename CaptureList::value_type *findCaptureById(const CaptureList &captures, uint32_t captureId) {
    for (const auto &capture : captures) {
        if (capture.captureId == captureId) return &capture;
    }
    return nullptr;
}

} // namespace LoRaSnifferStorage

#endif