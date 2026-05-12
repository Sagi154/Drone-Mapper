#pragma once

#include "mapping/IBuildingMap.h"

#include <filesystem>

namespace dmap {

/// Serializes map bounds plus every stored cell to path (truncates). Returns false on I/O open failure.
bool writeBuildingMap(const std::filesystem::path& path, const IBuildingMap& map);

}  // namespace dmap
