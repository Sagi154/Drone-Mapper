#pragma once

#include "common/Point3D.h"
#include "common/Types.h"

#include <cstdint>

namespace dmap {

/// Values stored in a building map cell (drone map or ground truth).
enum class MapValue : std::int8_t {
  Empty = 0,
  Occupied = 1,
  NotMapped = -1,
  OutOfBounds = -2,
};

/// Flight-volume box and grid resolution used to discretise coordinates.
/// Populated from mission or map input; positions outside return OutOfBounds.
struct MapBounds {
  LengthCm min_x{};
  LengthCm max_x{};
  LengthCm min_y{};
  LengthCm max_y{};
  LengthCm min_height{};
  LengthCm max_height{};
  /// Decimal places for X/Y quantization; cell size in cm = 10^(-xy_decimal_places).
  std::int32_t xy_decimal_places{4};
  /// Decimal places for height quantization.
  std::int32_t height_decimal_places{4};
};

/// One explicitly stored cell in a sparse building map.
struct CellEntry {
  /// World-space cell centre in centimetres (after quantization).
  Point3D position{};
  MapValue value{MapValue::NotMapped};
};

}  // namespace dmap
