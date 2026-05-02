#include "mapping/SparseKey.h"

#include <cmath>
#include <mp-units/systems/si/unit_symbols.h>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

std::size_t SparseKeyHash::operator()(SparseKey k) const noexcept {
  const std::size_t hx = static_cast<std::size_t>(k.qx);
  const std::size_t hy = static_cast<std::size_t>(k.qy);
  const std::size_t hz = static_cast<std::size_t>(k.qh);
  return hx ^ (hy << 1U) ^ (hz << 2U);
}

static std::int64_t quantizeScalar(double v, std::int32_t decimal_places) {
  const double scale = std::pow(10.0, static_cast<double>(decimal_places));
  return static_cast<std::int64_t>(std::llround(v * scale));
}

SparseKey quantizePoint(const Point3D& p, std::int32_t xy_decimal_places,
                        std::int32_t height_decimal_places) {
  const double x = p.x.numerical_value_in(su::cm);
  const double y = p.y.numerical_value_in(su::cm);
  const double h = p.height.numerical_value_in(su::cm);
  return SparseKey{quantizeScalar(x, xy_decimal_places), quantizeScalar(y, xy_decimal_places),
                   quantizeScalar(h, height_decimal_places)};
}

}  // namespace dmap
