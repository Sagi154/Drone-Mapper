#pragma once

#include "common/Point3D.h"

#include <cstddef>
#include <cstdint>

namespace dmap {

struct SparseKey {
  std::int64_t qx{};
  std::int64_t qy{};
  std::int64_t qh{};

  bool operator==(const SparseKey& o) const noexcept = default;
};

struct SparseKeyHash {
  std::size_t operator()(SparseKey k) const noexcept;
};

SparseKey quantizePoint(const Point3D& p, std::int32_t xy_decimal_places, std::int32_t height_decimal_places);

}  // namespace dmap
