#include "io/MapFileWriter.h"

#include <fstream>

namespace dmap {

bool writeBuildingMap(const std::filesystem::path& path, const IBuildingMap& map) {
  (void)map;
  std::ofstream out(path, std::ios::trunc);
  return static_cast<bool>(out);
}

}  // namespace dmap
