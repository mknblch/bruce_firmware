#pragma once

#include <FS.h>
#include <vector>

struct PinProfile {
    FS *fs;
    String path;
    String displayName;
};

std::vector<PinProfile> scanPinProfiles();
bool applyPinProfile(const PinProfile &profile, String &error);