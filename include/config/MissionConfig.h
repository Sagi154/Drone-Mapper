#pragma once

#include "common/Types.h"
#include "common/Point3D.h"
#include "sensors/IPositionSensor.h"

#include <cstdint>
#include <vector>

namespace dmap {

struct MissionConfig {
  // Mapping boundaries — defines the valid flight volume.
  LengthCm min_x{};
  LengthCm max_x{};
  LengthCm min_y{};
  LengthCm max_y{};
  LengthCm min_height{};
  LengthCm max_height{};

  // Where the drone starts. xy_angle defaults to 0 ("east" = +x direction)
  // if not specified in the mission file.
  DronePosition initial_position{};

  // Result map resolution: number of decimal places for X/Y and Height.
  std::int32_t xy_decimal_places{2};
  std::int32_t height_decimal_places{2};

  // Positions where the drone can recharge. May be empty.
  std::vector<Point3D> recharge_positions;
};

}  // namespace dmap
