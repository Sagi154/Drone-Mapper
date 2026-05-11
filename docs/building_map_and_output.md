# Drone building map and map output

This document describes the drone-side occupancy map, how it is written to disk,
and the automated tests that cover map behavior and scan-to-map integration.

The lidar simulator produces `LidarHit` records (azimuth, elevation, distance).
The map layer stores what the drone believes about space; it does not read the
simulator’s ground-truth map directly.

## What was implemented

- **`BuildingMap`** — sparse grid keyed by quantized coordinates from the
  mission’s decimal-place resolution. Cell values: `0` empty, `1` occupied,
  `-1` not mapped, `-2` outside the mission bounds.
- **`MapFileWriter`** — serializes a building map to `map_output.txt` using the
  same line grammar as map input: a `bounds` line, then `occupied` / `empty`
  lines only for cells that are explicitly stored.
- **Tests** — `tests/test_building_map.cpp` exercises get/set, bounds,
  quantization, `allCells()`, `bounds()`, and a write → read round trip via
  `writeBuildingMap` and `loadGroundTruthMap`. `tests/test_drone_algorithm.cpp`
  includes a case where one `DroneAlgorithm::tick()` runs a scan and updates the
  map (`Empty` along the beam, `Occupied` at the hit).

## Design decisions

- **Sparse storage** — unstored in-bounds cells report `NotMapped`, not empty, so
  “never observed” stays distinct from “observed free.”
- **Quantization** — XY and height steps come from
  `MissionConfig::xy_decimal_places` and `height_decimal_places`; writers and
  readers must use the same precision as the mission.
- **Shared file grammar** — drone output matches simulator input shape so
  comparison and tooling stay aligned.
- **Ray projection in the algorithm** — `DroneAlgorithm` walks each hit’s range
  (using the map’s grid step) to mark free space, then marks the endpoint
  occupied. World-space endpoints use `hitToWorldPoint` in
  [`src/common/MathUtils.h`](../src/common/MathUtils.h), aligned with the lidar
  mock’s azimuth/elevation convention.

## Public interfaces

- **`IBuildingMap`** — [`include/mapping/IBuildingMap.h`](../include/mapping/IBuildingMap.h)
- **`writeBuildingMap`** — [`src/io/MapFileWriter.h`](../src/io/MapFileWriter.h)
- **`MapValue` / `MapBounds`** — [`include/mapping/MapTypes.h`](../include/mapping/MapTypes.h)

## Source files

| Role | Path |
|------|------|
| Map implementation | [`src/mapping/BuildingMap.cpp`](../src/mapping/BuildingMap.cpp), [`src/mapping/BuildingMap.h`](../src/mapping/BuildingMap.h) |
| Map output | [`src/io/MapFileWriter.cpp`](../src/io/MapFileWriter.cpp) |
