#pragma once

#include <algorithm>
#include <cctype>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace dmap::config_parse {

/// Remove leading/trailing whitespace from a line.
/// @param s  Input string to normalize.
/// @return   Trimmed string with no outer whitespace.
inline std::string trim(std::string s) {
  auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [&](unsigned char c) {
            return !is_space(c);
          }));
  s.erase(std::find_if(s.rbegin(), s.rend(), [&](unsigned char c) {
            return !is_space(c);
          }).base(),
          s.end());
  return s;
}

/// Drop inline '#' comments and trim the remaining text.
/// @param line  Raw input line from a config-like file.
/// @return      Clean line content, or empty if only comment/whitespace.
inline std::string stripCommentAndTrim(std::string line) {
  const auto hash = line.find('#');
  if (hash != std::string::npos) {
    line.erase(hash);
  }
  return trim(std::move(line));
}

/// Parse a "key = value" line into a key/value pair.
/// @param raw_line  Raw line from a config file.
/// @return          Parsed pair when valid, nullopt otherwise.
inline std::optional<std::pair<std::string, std::string>> parseKeyValueLine(
    const std::string& raw_line) {
  const std::string line = stripCommentAndTrim(raw_line);
  if (line.empty()) {
    return std::nullopt;
  }

  const auto eq = line.find('=');
  if (eq == std::string::npos) {
    return std::nullopt;
  }

  std::string key = trim(line.substr(0, eq));
  std::string value = trim(line.substr(eq + 1));
  if (key.empty() || value.empty()) {
    return std::nullopt;
  }
  return std::make_pair(std::move(key), std::move(value));
}

/// Split a line into whitespace-delimited tokens.
/// @param line  Input line to tokenize.
/// @return      Ordered list of non-empty tokens.
inline std::vector<std::string> splitWhitespace(const std::string& line) {
  std::vector<std::string> parts;
  std::string token;
  for (char ch : line) {
    if (std::isspace(static_cast<unsigned char>(ch)) != 0) {
      if (!token.empty()) {
        parts.push_back(std::move(token));
        token.clear();
      }
    } else {
      token.push_back(ch);
    }
  }
  if (!token.empty()) {
    parts.push_back(std::move(token));
  }
  return parts;
}

}  // namespace dmap::config_parse
