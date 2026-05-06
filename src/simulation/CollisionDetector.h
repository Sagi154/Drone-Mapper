#pragma once

#include "common/Point3D.h"
#include "config/DroneConfig.h"
#include "sensors/IPositionSensor.h"

namespace dmap {

class SimulationState;

class CollisionDetector {
 public:
  // Lightweight constructor for point-only checks (intersectsOccupied).
  // Footprint methods are not available with this constructor.
  // Callers: LidarMock::scan, tests/test_collision_detector.cpp
  explicit CollisionDetector(const SimulationState& state);

  // Full constructor required for footprint / face checks.
  // step_xy_cm and step_height_cm should equal the map cell size
  // (10^(-decimal_places) from MissionConfig).
  CollisionDetector(const SimulationState& state, DroneConfig drone_cfg,
                    double step_xy_cm, double step_height_cm);

  // Returns true if the single cell at `at` is Occupied in the ground-truth map.
  //
  // Callers:
  //   intersectsFootprint (below) — inner loop
  //   LidarMock::scan             (LidarMock.cpp)
  //   tests/test_collision_detector.cpp
  bool intersectsOccupied(const Point3D& at) const;

  // Returns true if ANY cell inside the drone's full oriented bounding box at
  // `pos` is Occupied. Use for static placement checks (e.g. initial position).
  bool intersectsFootprint(const DronePosition& pos) const;

  // Checks only the forward face of the footprint (dl = +half_length).
  // Use during advance: only the face entering new territory needs checking;
  // the rest of the box was already clear at the previous step.
  //
  // Caller: MovementMock::advance (MovementMock.cpp)
  bool intersectsForwardFace(const DronePosition& pos) const;

  // Checks only the top face (upward=true, dh = +half_height) or bottom face
  // (upward=false, dh = -half_height) of the footprint.
  // Use during elevate: only the face entering new territory needs checking.
  //
  // Caller: MovementMock::elevate (MovementMock.cpp)
  bool intersectsElevateFace(const DronePosition& pos, bool upward) const;

 private:
  const SimulationState& state_;
  DroneConfig drone_cfg_{};
  double step_xy_cm_{1.0};
  double step_height_cm_{1.0};
};

}  // namespace dmap
