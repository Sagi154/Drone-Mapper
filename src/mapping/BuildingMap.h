#pragma once

#include "config/MissionConfig.h"
#include "mapping/IBuildingMap.h"
#include "mapping/SparseKey.h"

#include <unordered_map>

namespace dmap {

class BuildingMap final : public IBuildingMap {
 public:
  explicit BuildingMap(MissionConfig mission);

  MapValue get(const Point3D& p) const override;
  void set(const Point3D& p, MapValue v) override;

 private:
  MissionConfig mission_{};
  mutable std::unordered_map<SparseKey, MapValue, SparseKeyHash> cells_{};

  bool inBounds(const Point3D& p) const;
};

}  // namespace dmap
