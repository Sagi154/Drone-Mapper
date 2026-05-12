#pragma once

#include "common/Point3D.h"
#include "mapping/MapTypes.h"

#include <vector>

namespace dmap {

/// Sparse drone-side map: get/set by world point, plus bounds and iteration for I/O.
class IBuildingMap {
 public:
  virtual ~IBuildingMap() = default;

  virtual MapValue get(const Point3D& p) const = 0;
  virtual void set(const Point3D& p, MapValue v) = 0;

  /// Mission mapping volume and grid resolution (same semantics as map file bounds line).
  virtual MapBounds bounds() const = 0;

  /// Every cell explicitly stored in the sparse map (empty or occupied).
  virtual std::vector<CellEntry> allCells() const = 0;
};

}  // namespace dmap
