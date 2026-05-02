#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace dmap {

class ErrorLogger {
 public:
  void add(std::string line);
  void flushIfNeeded(const std::filesystem::path& input_errors_txt);

 private:
  std::vector<std::string> lines_;
};

}  // namespace dmap
