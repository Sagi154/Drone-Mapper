#include "mapping/Scorer.h"

#include "mapping/IBuildingMap.h"

namespace dmap {

double Scorer::scorePercent(const IBuildingMap& produced, const IBuildingMap& reference) const {
  (void)produced;
  (void)reference;
  return 0.0;
}

}  // namespace dmap
