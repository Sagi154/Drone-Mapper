#pragma once

#include "common/Point3D.h"
#include "mapping/MapTypes.h"
#include "sensors/IPositionSensor.h"

#include <string>
#include <unordered_map>

namespace dmap {

// Axis-aligned bounding box that defines the valid flight volume,
// together with the grid resolution that discretises it.
// Loaded from MissionConfig; positions outside return OutOfBounds.
struct MapBounds {
  LengthCm min_x{};
  LengthCm max_x{};
  LengthCm min_y{};
  LengthCm max_y{};
  LengthCm min_height{};
  LengthCm max_height{};
  // Decimal places for X/Y and Height coordinates (from MissionConfig).
  // Cell size in cm = 10^(-decimal_places).
  // e.g. xy_decimal_places=2 → cell is 0.01 cm wide.
  std::int32_t xy_decimal_places{4};
  std::int32_t height_decimal_places{4};
};

class SimulationState {
 public:
  // --- Drone position ---
  DronePosition dronePosition() const { return pos_; }
  void setDronePosition(DronePosition p) { pos_ = p; }

  // --- Map bounds ---
  // Call once after loading the map file. Required for truthValue() to
  // distinguish OutOfBounds from Empty.
  void setMapBounds(const MapBounds& bounds);
  bool hasBounds() const { return has_bounds_; }
  MapBounds mapBounds() const { return bounds_; }

  // --- Ground-truth cells ---
  // Clears all stored cells (does not reset bounds).
  void clearGroundTruth();
  // Store what a cell actually is in the real world (called by MapFileReader).
  void setTruthCell(const Point3D& p, MapValue v);
  // Query what a cell actually is.
  // Returns OutOfBounds if bounds are set and p is outside them,
  // Empty if p is inside but was never explicitly stored, otherwise the
  // stored value.
  //
  // Callers:
  //   CollisionDetector::intersectsOccupied  (CollisionDetector.cpp)
  //   LidarMock::scan                        (LidarMock.cpp)
  MapValue truthValue(const Point3D& p) const;

 private:
  DronePosition pos_{};
  std::unordered_map<std::string, MapValue> truth_;
  MapBounds bounds_{};
  bool has_bounds_{false};

  bool isInBounds(const Point3D& p) const;
  // Rounds each coordinate to the nearest grid cell and returns a string key.
  // Rounding ensures that floating-point drift from movement math (e.g.
  // 100.00001 cm vs 100.00000 cm) always resolves to the same cell.
  std::string truthKey(const Point3D& p) const;
};

}  // namespace dmap
