#include "io/ErrorLogger.h"

#include <fstream>

namespace dmap {

void ErrorLogger::add(std::string line) { lines_.push_back(std::move(line)); }

void ErrorLogger::flushIfNeeded(const std::filesystem::path& input_errors_txt) {
  if (lines_.empty()) {
    return;
  }
  std::ofstream out(input_errors_txt, std::ios::trunc);
  for (const auto& l : lines_) {
    out << l << '\n';
  }
}

}  // namespace dmap
