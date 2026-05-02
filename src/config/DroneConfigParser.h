#pragma once

#include "config/DroneConfig.h"

#include <filesystem>

namespace dmap {

DroneConfig parseDroneConfig(const std::filesystem::path& path);

}  // namespace dmap
