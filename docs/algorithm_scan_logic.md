# Scan Logic — `DroneAlgorithm::fullScan()`

## Purpose

Before the drone plans its next move, it must confirm the space around it by
firing the lidar in every direction. This is done by `fullScan()`, called at
the start of every Scanning phase.

A single `scan()` call only fires a cone of beams in the drone's current
facing direction. Cells between beams (blind spots) remain `NotMapped`. The
sphere-safe passability check in BFS requires every cell within
`min_passable_radius` to be `Empty`, so blind spots must be eliminated before
planning — otherwise the BFS finds no passable cells.

## How it works

`fullScan()` performs a **full spherical sweep** from the drone's current
position (position is fixed for all shots in the sweep):

```
elevation tier -90° (straight down)
  → azimuth sweep 0° – 360°
elevation tier -90° + el_step
  → azimuth sweep 0° – 360°
...
elevation tier +90° (straight up)
  → azimuth sweep 0° – 360°
```

Each shot calls `lidar_.scan(azimuth_offset, elevation_angle)` and fuses the
result into the drone's map via `applyLidarHitsToMap`.

## Angular step sizing

The angular step `el_step` is derived so that at `z_min` distance the arc
between two adjacent aim directions is at most one grid cell:

```
cell_cm  = min(xy_cell_size, height_cell_size)   // finest grid resolution
denom    = z_min  (or 1 cm if z_min == 0)
el_step  = atan(cell_cm / denom)  in degrees
```

This guarantees no grid cell is skipped between beam paths at the drone's
minimum operational range. The same step is used for both elevation and
azimuth at the equatorial belt.

## Adaptive azimuth step near the poles

At elevation ±90° the latitude circle degenerates to a point — a single shot
covers the full azimuth. At intermediate elevations the circle radius shrinks
proportionally. The azimuth step is widened to maintain the same physical
ground spacing:

```
az_step = el_step / cos(el)     (when cos(el) > 1e-6)
az_step = 360°                  (at the poles)
```

## Result

After `fullScan()`, every grid cell within lidar range in every 3D direction
has been swept by at least one beam. Cells with no obstacle return `Empty`
along the full ray; cells with an obstacle return `Occupied` at the hit and
`Empty` along the ray up to it. Only cells outside the lidar cone at all
elevations and azimuths may still be `NotMapped` — those are beyond the
map boundaries or in lidar-shadow behind obstacles.

## Relevant source

- `src/algorithm/DroneAlgorithm.cpp` — `fullScan()` implementation
- `src/algorithm/DroneAlgorithm.h` — `fullScan()` declaration
- `include/sensors/ILidarSensor.h` — `scan(xy_offset, height_angle)` interface
- `src/simulation/LidarMock.cpp` — applies the offsets to world-space rays
