// MovementMock.h
// Simulates the drone's movement inside the SimulationState.
// Enforces all physical constraints from DroneConfig (max distances per
// command, drone body dimensions) and rejects any move that would cause
// the drone to occupy or pass through an Occupied cell.

#pragma once

#include "config/DroneConfig.h"
#include "config/MissionConfig.h"
#include "drivers/IMovementDriver.h"
#include "simulation/CollisionDetector.h"
#include "simulation/SimulationState.h"

namespace dmap {

// Simulated movement driver.  Implements IMovementDriver so the drone
// algorithm can call it without knowing it is talking to a simulation.
// Every move is validated against DroneConfig limits and the ground-truth
// map before the drone's position in SimulationState is updated.
class MovementMock final : public IMovementDriver {
 public:
  // Builds the mock with the drone's capability limits (drone_cfg) and the
  // mission resolution (mission_cfg), which determines the step size used
  // when ray-marching through the map to detect obstacles along the path.
  MovementMock(SimulationState& state, DroneConfig drone_cfg,
               const MissionConfig& mission_cfg);

  // Turns the drone left or right by `angle` degrees.
  // Negative angles are allowed (spec-permitted).
  // Rejected (no-op) if |angle| exceeds max_rotate_per_command.
  void rotate(TurnDirection direction, AngleDeg angle) override;

  // Moves the drone forward (positive) or backward (negative) by `distance` cm.
  // Rejected if: |distance| exceeds max_advance_per_command,
  // or any cell along the path (checked step-by-step) is Occupied.
  void advance(LengthCm distance) override;

  // Changes the drone's altitude by `distance` cm (positive = up, negative = down).
  // Rejected if: |distance| exceeds max_elevate_per_command,
  // or any cell along the vertical path is Occupied.
  void elevate(LengthCm distance) override;

 private:
  SimulationState& state_;
  DroneConfig cfg_{};
  // Grid cell sizes in cm, derived from mission resolution (10^(-decimal_places)).
  // Declared before detector_ to guarantee initialisation order.
  double step_xy_cm_{1.0};
  double step_height_cm_{1.0};
  CollisionDetector detector_;
};

}  // namespace dmap
