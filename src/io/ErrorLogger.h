#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace dmap {

class ErrorLogger {
 public:
  void add(std::string line);
  void flushIfNeeded(const std::filesystem::path& input_errors_txt);
  [[nodiscard]] bool empty() const { return lines_.empty(); }
  [[nodiscard]] std::size_t size() const { return lines_.size(); }
  [[nodiscard]] const std::vector<std::string>& lines() const { return lines_; }

 private:
  std::vector<std::string> lines_;
};

}  // namespace dmap
