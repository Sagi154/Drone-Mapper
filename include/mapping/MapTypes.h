#pragma once

#include <cstdint>

namespace dmap {

enum class MapValue : std::int8_t {
  Empty = 0,
  Occupied = 1,
  NotMapped = -1,
  OutOfBounds = -2,
};

}  // namespace dmap
