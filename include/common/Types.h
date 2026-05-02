#pragma once

#include <mp-units/systems/si.h>
#include <mp-units/systems/si/unit_symbols.h>

namespace dmap {

namespace su = mp_units::si::unit_symbols;

using LengthCm = decltype(1.0 * su::cm);
using AngleDeg = decltype(1.0 * su::deg);

}  // namespace dmap
