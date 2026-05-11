#pragma once

#include "config/MissionConfig.h"
#include "mapping/IBuildingMap.h"
#include "mapping/SparseKey.h"

#include <unordered_map>
#include <vector>

namespace dmap {

class BuildingMap final : public IBuildingMap {
 public:
  explicit BuildingMap(MissionConfig mission);

  MapValue get(const Point3D& p) const override;
  void set(const Point3D& p, MapValue v) override;

  MapBounds bounds() const override;
  std::vector<CellEntry> allCells() const override;

 private:
  MissionConfig mission_{};
  mutable std::unordered_map<SparseKey, MapValue, SparseKeyHash> cells_{};

  bool inBounds(const Point3D& p) const;
};

}  // namespace dmap
