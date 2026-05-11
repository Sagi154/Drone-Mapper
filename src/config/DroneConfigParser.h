#pragma once

#include "config/DroneConfig.h"
#include "io/ErrorLogger.h"

#include <filesystem>

namespace dmap {

DroneConfig parseDroneConfig(const std::filesystem::path& path, ErrorLogger& logger);

}  // namespace dmap
