#pragma once

#include "common/Point3D.h"
#include "mapping/MapTypes.h"

namespace dmap {

class IBuildingMap {
 public:
  virtual ~IBuildingMap() = default;

  virtual MapValue get(const Point3D& p) const = 0;
  virtual void set(const Point3D& p, MapValue v) = 0;
};

}  // namespace dmap
