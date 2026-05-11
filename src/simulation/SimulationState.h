// SimulationState.h
// The single source of truth for the simulated world.
// Holds two things: the drone's current position, and the complete
// ground-truth cell map (what cells are really Empty or Occupied).
// This is the "hidden world" that only the simulation side can read;
// the drone's own BuildingMap is built separately from lidar scans.

#pragma once

#include "common/Point3D.h"
#include "mapping/MapTypes.h"
#include "sensors/IPositionSensor.h"

#include <string>
#include <unordered_map>

namespace dmap {

// Describes the valid flight volume and the grid resolution used to
// discretise it.  Populated from MissionConfig after loading the map file.
// Any position outside the box returns MapValue::OutOfBounds from truthValue().
struct MapBounds {
  LengthCm min_x{};
  LengthCm max_x{};
  LengthCm min_y{};
  LengthCm max_y{};
  LengthCm min_height{};
  LengthCm max_height{};
  // Number of decimal places used to round coordinates to a grid cell.
  // Cell size in cm = 10^(-decimal_places).
  // e.g. xy_decimal_places=2 → each cell is 0.01 cm wide.
  std::int32_t xy_decimal_places{4};
  std::int32_t height_decimal_places{4};
};

// Owns the entire simulated world state: drone position + ground-truth map.
// Only simulation components (CollisionDetector, LidarMock, MovementMock)
// read or write this object.  The drone algorithm never touches it directly.
class SimulationState {
 public:
  // --- Drone position ---

  // Returns the drone's current position (center point + heading angle).
  DronePosition dronePosition() const { return pos_; }

  // Overwrites the drone's position.  Called by MovementMock after a
  // successful move, and by the test harness to set up scenarios.
  void setDronePosition(DronePosition p) { pos_ = p; }

  // --- Map bounds ---

  // Stores the flight-volume box and grid resolution.
  // Must be called once (after MapFileReader loads the map) before any
  // truthValue() queries, so the system can tell OutOfBounds from Empty.
  void setMapBounds(const MapBounds& bounds);

  // Returns true if setMapBounds has been called.
  bool hasBounds() const { return has_bounds_; }

  // Returns the currently stored bounds (valid only if hasBounds() is true).
  MapBounds mapBounds() const { return bounds_; }

  // --- Ground-truth cells ---

  // Removes all stored cell values.  Bounds are preserved so that the
  // flight volume remains valid after a map reload.
  void clearGroundTruth();

  // Records the true value of a cell at position p (Empty or Occupied).
  // Called by MapFileReader when loading the map from disk.
  void setTruthCell(const Point3D& p, MapValue v);

  // Returns the true value of the cell that contains position p.
  //   OutOfBounds — bounds are set and p lies outside the flight volume.
  //   Empty       — p is inside the volume but no cell was explicitly stored.
  //   Occupied    — an obstacle was stored at that cell by setTruthCell.
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

  // Returns true if p is inside the flight-volume box.
  bool isInBounds(const Point3D& p) const;

  // Converts p to a string hash key by rounding each coordinate to the
  // nearest grid cell.  Rounding absorbs floating-point drift so that
  // positions within the same physical cell always produce the same key
  // (e.g. 100.00001 cm and 100.00000 cm both map to the same cell).
  std::string truthKey(const Point3D& p) const;
};

}  // namespace dmap
