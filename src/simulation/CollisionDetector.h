// CollisionDetector.h
// Checks whether a position or the drone's physical body overlaps with an
// Occupied cell in the ground-truth map.
//
// The drone is modelled as a perfect sphere of radius min_passable_radius.
// Because a sphere has no preferred orientation, its collision footprint is
// identical regardless of heading — rotation never changes which cells are
// tested.
//
// Two usage modes:
//   1. Point-only (lightweight constructor): just asks "is this exact cell
//      occupied?" — used by LidarMock when ray-marching beams.
//   2. Full footprint (full constructor): samples the drone's spherical
//      volume at every grid cell and returns true if any cell is Occupied —
//      used by MovementMock to prevent the drone from entering narrow gaps
//      or tunnelling through thin walls.

#pragma once

#include "common/Point3D.h"
#include "config/DroneConfig.h"
#include "sensors/IPositionSensor.h"

namespace dmap {

class SimulationState;

// Queries the ground-truth map to detect collisions with the drone's body
// or with a single point in space.
class CollisionDetector {
 public:
  // Lightweight constructor — only intersectsOccupied() is available.
  // Used when the caller only needs single-cell point checks (e.g. LidarMock).
  explicit CollisionDetector(const SimulationState& state);

  // Full constructor — enables footprint and face checks.
  // step_xy_cm / step_height_cm are the grid cell sizes derived from
  // MissionConfig (10^(-decimal_places) cm); they control how densely the
  // bounding box is sampled so no cell is ever skipped.
  // The bounding box span is always rounded outward (ceiling), so every grid
  // cell the drone physically overlaps is guaranteed to be tested — rounding
  // inward could silently miss an occupied cell at the drone's edge.
  CollisionDetector(const SimulationState& state, DroneConfig drone_cfg,
                    double step_xy_cm, double step_height_cm);

  // Returns true if the cell at position `at` is Occupied in the ground-truth map.
  // Used as the inner check for all ray-march and footprint loops.
  //
  // Callers:
  //   intersectsFootprint / intersectsForwardFace / intersectsElevateFace (below)
  //   LidarMock::scan   (LidarMock.cpp)
  //   tests/test_collision_detector.cpp
  bool intersectsOccupied(const Point3D& at) const;

  // Returns true if ANY cell inside the drone's spherical volume at position
  // `pos` is Occupied.
  // Used for static placement checks (e.g. verifying the initial drone position).
  bool intersectsFootprint(const DronePosition& pos) const;

  // Returns true if any cell in the drone's forward hemisphere (the half of
  // the sphere whose centre-to-sample vector has a non-negative dot product
  // with the current heading) is Occupied.
  // Used by MovementMock::advance — only the leading hemisphere needs
  // checking because the trailing half was already clear one step ago.
  bool intersectsForwardFace(const DronePosition& pos) const;

  // Returns true if any cell in the drone's top hemisphere (upward=true) or
  // bottom hemisphere (upward=false) is Occupied.
  // Used by MovementMock::elevate — only the leading hemisphere is checked for
  // the same reason as intersectsForwardFace.
  bool intersectsElevateFace(const DronePosition& pos, bool upward) const;

 private:
  const SimulationState& state_;
  DroneConfig drone_cfg_{};
  double step_xy_cm_{1.0};      // sampling step across XY dimensions
  double step_height_cm_{1.0};  // sampling step across the height dimension
};

}  // namespace dmap
