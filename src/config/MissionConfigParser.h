#pragma once

#include "config/MissionConfig.h"
#include "io/ErrorLogger.h"

#include <filesystem>

namespace dmap {

MissionConfig parseMissionConfig(const std::filesystem::path& path, ErrorLogger& logger);

}  // namespace dmap
