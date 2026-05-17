// ExplorationFrontier.h
// Finds the nearest reachable frontier in the drone's discovered map using BFS.
// A frontier is a confirmed-empty cell adjacent to at least one NotMapped cell —
// moving there and scanning may reveal new space.
// Only sphere-safe cells are considered passable: every grid cell within
// drone_radius of a candidate cell centre must be Empty.

#pragma once

#include "common/Point3D.h"
#include "common/Types.h"
#include "mapping/IBuildingMap.h"

#include <vector>

namespace dmap {

/// Path returned by ExplorationFrontier::findPath().
struct PathResult {
  bool found{false};
  /// Waypoints from start (exclusive) to the frontier cell (inclusive),
  /// each a grid-aligned world-space point in cm.
  std::vector<Point3D> path{};
};

/// Finds the nearest reachable frontier from a given start position via BFS
/// through sphere-safe confirmed-empty cells.
class ExplorationFrontier {
 public:
  /// BFS from `start` through the drone's map.
  ///
  /// Passability rule: a cell is passable only if every grid cell within
  /// `drone_radius` of its centre is Empty in `map`. NotMapped or Occupied
  /// neighbours within the radius block movement.
  ///
/// A frontier is a passable cell with at least one axis-aligned neighbour
/// that is not sphere-passable (unknown, occupied, or out of bounds).
  ///
  /// Neighbour expansion order is fixed (+X, -X, +Y, -Y, +Height, -Height)
  /// for determinism.
  ///
  /// @param map          The drone's discovered building map.
  /// @param start        World-space start position (drone centre, in cm).
  /// @param drone_radius Sphere radius used for the passability check (cm).
  /// @return PathResult with found=true and the path, or found=false if no
  ///         reachable frontier exists.
  PathResult findPath(const IBuildingMap& map, const Point3D& start,
                      LengthCm drone_radius) const;
};

}  // namespace dmap
