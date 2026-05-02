#pragma once

namespace dmap {

class IBuildingMap;

class Scorer {
 public:
  double scorePercent(const IBuildingMap& produced, const IBuildingMap& reference) const;
};

}  // namespace dmap
