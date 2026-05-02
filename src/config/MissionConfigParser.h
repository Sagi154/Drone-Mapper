#pragma once

#include "config/MissionConfig.h"

#include <filesystem>

namespace dmap {

MissionConfig parseMissionConfig(const std::filesystem::path& path);

}  // namespace dmap
