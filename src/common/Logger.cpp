#include "common/Logger.h"

#include <iostream>

namespace dmap::log {

void info(std::string_view message) { std::cerr << "[dmap] " << message << '\n'; }

}  // namespace dmap::log
