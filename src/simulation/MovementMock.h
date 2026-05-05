#pragma once

#include "config/DroneConfig.h"
#include "config/MissionConfig.h"
#include "drivers/IMovementDriver.h"
#include "simulation/CollisionDetector.h"
#include "simulation/SimulationState.h"

namespace dmap {

class MovementMock final : public IMovementDriver {
 public:
  MovementMock(SimulationState& state, DroneConfig drone_cfg,
               const MissionConfig& mission_cfg);

  void rotate(TurnDirection direction, AngleDeg angle) override;
  // Moves forward in the direction the drone currently faces.
  // Rejected if: distance is negative, exceeds max_advance_per_command,
  // or the destination cell is occupied.
  void advance(LengthCm distance) override;
  // Changes altitude. Positive = up, negative = down.
  // Rejected if: |distance| exceeds max_elevate_per_command,
  // or the destination cell is occupied.
  void elevate(LengthCm distance) override;

 private:
  SimulationState& state_;
  DroneConfig cfg_{};
  // Step sizes derived from mission resolution: 10^(-decimal_places) cm.
  // Declared before detector_ so they are initialised first and can be
  // forwarded to the CollisionDetector constructor.
  double step_xy_cm_{1.0};
  double step_height_cm_{1.0};
  CollisionDetector detector_;
};

}  // namespace dmap
