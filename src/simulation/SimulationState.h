#pragma once

#include "common/Point3D.h"
#include "mapping/MapTypes.h"
#include "sensors/IPositionSensor.h"

#include <string>
#include <unordered_map>

namespace dmap {

class SimulationState {
 public:
  DronePosition dronePosition() const { return pos_; }
  void setDronePosition(DronePosition p) { pos_ = p; }

  void clearGroundTruth();
  void setTruthCell(const Point3D& p, MapValue v);
  MapValue truthValue(const Point3D& p) const;

 private:
  DronePosition pos_{};
  std::unordered_map<std::string, MapValue> truth_;

  static std::string truthKey(const Point3D& p);
};

}  // namespace dmap
