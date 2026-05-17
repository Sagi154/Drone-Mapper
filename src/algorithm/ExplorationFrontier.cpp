// ExplorationFrontier.cpp
// BFS-based frontier search through the drone's confirmed-empty map cells.
// Passability requires the drone's full sphere (radius = drone_radius) to fit:
// every grid cell within drone_radius of a candidate centre must be Empty.
// NotMapped counts as impassable — it means no information, not "safe".

#include "algorithm/ExplorationFrontier.h"

#include "common/MathUtils.h"
#include "mapping/MapTypes.h"
#include "mapping/SparseKey.h"

#include <mp-units/systems/si/unit_symbols.h>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace dmap {

namespace {

namespace su = mp_units::si::unit_symbols;

// Convert a SparseKey back to a world-space Point3D using grid step sizes.
Point3D keyToPoint(SparseKey k, double xy_step, double h_step) {
  return {(static_cast<double>(k.qx) * xy_step) * su::cm,
          (static_cast<double>(k.qy) * xy_step) * su::cm,
          (static_cast<double>(k.qh) * h_step)  * su::cm};
}

// Returns true if every grid cell whose centre lies within drone_radius of
// `centre` is Empty in `map`. Any NotMapped, Occupied or OutOfBounds cell
// within the sphere makes this return false.
bool isSpherePassable(const IBuildingMap& map, const Point3D& centre,
                      double radius_cm, double xy_step, double h_step) {
  const double cx = centre.x.numerical_value_in(su::cm);
  const double cy = centre.y.numerical_value_in(su::cm);
  const double ch = centre.height.numerical_value_in(su::cm);

  // Number of grid steps that fit within the radius in each axis.
  const int rx = static_cast<int>(std::ceil(radius_cm / xy_step));
  const int rh = static_cast<int>(std::ceil(radius_cm / h_step));

  for (int dx = -rx; dx <= rx; ++dx) {
    for (int dy = -rx; dy <= rx; ++dy) {
      for (int dz = -rh; dz <= rh; ++dz) {
        const double ox = dx * xy_step;
        const double oy = dy * xy_step;
        const double oz = dz * h_step;
        // Only test cells whose centre actually lies within the sphere.
        if (ox * ox + oy * oy + oz * oz > radius_cm * radius_cm) continue;

        const Point3D p{(cx + ox) * su::cm, (cy + oy) * su::cm, (ch + oz) * su::cm};
        if (map.get(p) != MapValue::Empty) return false;
      }
    }
  }
  return true;
}

// Returns true when `cell` borders the passable region: some axis-aligned
// neighbour centre cannot host the drone sphere (NotMapped, Occupied, or
// OutOfBounds). With radius >= grid step, a grid-adjacent NotMapped cell is
// always non-passable, so this also marks the edge of known-empty space.
bool isFrontier(const IBuildingMap& map, const Point3D& cell, double radius_cm,
                double xy_step, double h_step) {
  const double cx = cell.x.numerical_value_in(su::cm);
  const double cy = cell.y.numerical_value_in(su::cm);
  const double ch = cell.height.numerical_value_in(su::cm);

  // Fixed neighbour order: +X, -X, +Y, -Y, +Height, -Height.
  const Point3D neighbours[6] = {
    {(cx + xy_step) * su::cm, cy * su::cm, ch * su::cm},
    {(cx - xy_step) * su::cm, cy * su::cm, ch * su::cm},
    {cx * su::cm, (cy + xy_step) * su::cm, ch * su::cm},
    {cx * su::cm, (cy - xy_step) * su::cm, ch * su::cm},
    {cx * su::cm, cy * su::cm, (ch + h_step) * su::cm},
    {cx * su::cm, cy * su::cm, (ch - h_step) * su::cm},
  };
  for (const auto& n : neighbours) {
    if (!isSpherePassable(map, n, radius_cm, xy_step, h_step)) {
      return true;
    }
  }
  return false;
}

}  // namespace

PathResult ExplorationFrontier::findPath(const IBuildingMap& map,
                                         const Point3D& start,
                                         LengthCm drone_radius) const {
  const MapBounds b = map.bounds();
  const double xy_step  = decimalPlacesToStep(b.xy_decimal_places);
  const double h_step   = decimalPlacesToStep(b.height_decimal_places);
  const double radius_cm = drone_radius.numerical_value_in(su::cm);

  const SparseKey start_key = quantizePoint(start, b.xy_decimal_places, b.height_decimal_places);

  // parent_of[key] = the key BFS came from; also serves as visited set.
  std::unordered_map<SparseKey, SparseKey, SparseKeyHash> parent_of;
  std::queue<SparseKey> queue;

  const Point3D start_pt = keyToPoint(start_key, xy_step, h_step);
  if (!isSpherePassable(map, start_pt, radius_cm, xy_step, h_step)) {
    return {};
  }

  parent_of[start_key] = start_key;
  queue.push(start_key);

  // Fixed axis-aligned neighbour offsets: +X, -X, +Y, -Y, +Height, -Height.
  struct Offset { int dx, dy, dh; };
  constexpr Offset kOffsets[6] = {
    { 1,  0,  0}, {-1,  0,  0},
    { 0,  1,  0}, { 0, -1,  0},
    { 0,  0,  1}, { 0,  0, -1},
  };

  while (!queue.empty()) {
    const SparseKey cur = queue.front();
    queue.pop();

    const Point3D cur_pt = keyToPoint(cur, xy_step, h_step);

    if (cur != start_key &&
        isFrontier(map, cur_pt, radius_cm, xy_step, h_step)) {
      // Reconstruct path from start (exclusive) to frontier (inclusive).
      PathResult result;
      result.found = true;
      SparseKey step = cur;
      while (!(step == start_key)) {
        result.path.push_back(keyToPoint(step, xy_step, h_step));
        step = parent_of.at(step);
      }
      std::reverse(result.path.begin(), result.path.end());
      return result;
    }

    for (const auto& off : kOffsets) {
      const SparseKey nb{cur.qx + off.dx, cur.qy + off.dy, cur.qh + off.dh};
      if (parent_of.count(nb)) continue;

      const Point3D nb_pt = keyToPoint(nb, xy_step, h_step);
      if (!isSpherePassable(map, nb_pt, radius_cm, xy_step, h_step)) continue;

      parent_of[nb] = cur;
      queue.push(nb);
    }
  }

  return {};
}

}  // namespace dmap
