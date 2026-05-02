#pragma once

#include "mapping/IBuildingMap.h"

#include <filesystem>

namespace dmap {

bool writeBuildingMap(const std::filesystem::path& path, const IBuildingMap& map);

}  // namespace dmap
