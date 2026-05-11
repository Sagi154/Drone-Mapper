# Simulation Layer — Overview

The `src/simulation/` folder is owned by **Partner A**.  
It contains everything that represents the **fake physical world** the drone flies in during development and testing. The drone algorithm never reads these files directly — it only talks to the interfaces (`IMovementDriver`, `IPositionSensor`, `ILidarSensor`) that the simulation implements behind the scenes.

---

## How the pieces fit together

```
                  ┌─────────────────────────────────┐
                  │         SimulationState          │
                  │  • drone position (x, y, h, θ)  │
                  │  • ground-truth cell map         │
                  │  • flight-volume bounds          │
                  │  • grid resolution               │
                  └────────────────┬────────────────┘
                                   │  read / write
              ┌────────────────────┼─────────────────────┐
              │                    │                      │
    ┌─────────▼──────┐   ┌─────────▼──────┐   ┌─────────▼──────┐
    │  MovementMock  │   │  PositionMock  │   │   LidarMock    │
    │ IMovementDriver│   │IPositionSensor │   │ ILidarSensor   │
    └────────┬───────┘   └────────────────┘   └────────┬───────┘
             │                                          │
    ┌────────▼───────┐                       ┌──────────▼──────────┐
    │CollisionDetect.│                       │ LidarBeamCalculator │
    └────────────────┘                       └─────────────────────┘
```

The **DroneAlgorithm** only ever calls the three interfaces. The simulation objects implement those interfaces and share a single `SimulationState` instance.

---

## Two maps — do not confuse them

| | `SimulationState::truth_` | `BuildingMap::cells_` |
|---|---|---|
| Owner | Partner A (simulation) | Partner B (algorithm) |
| Contents | Complete ground-truth: every wall loaded from the map file | Drone's discovered map: starts all `NotMapped`, filled in by lidar scans |
| Default for unknown cell | `Empty` (open air — only walls are stored) | `NotMapped` (not yet scanned) |
| Out-of-bounds | `OutOfBounds` | `OutOfBounds` |
| Who reads it | `CollisionDetector`, `LidarMock` | `DroneAlgorithm`, `Scorer` |
| Who writes it | `MapFileReader` (via `setTruthCell`) | `DroneAlgorithm` (via `BuildingMap::set`) |

---

## File-by-file reference

### `SimulationState.h / .cpp`

**What it is:** The central data store for the simulated world.

**What it holds:**
- The drone's current position (`DronePosition`: x, y, height, heading angle).
- The complete ground-truth cell map — a sparse hash map from a string grid-cell key to `MapValue` (`Empty` or `Occupied`). Only walls are stored; everything else is implicitly `Empty`.
- A `MapBounds` struct describing the valid flight volume and the grid resolution (`xy_decimal_places`, `height_decimal_places` from `MissionConfig`).

**Key design decision — coordinate rounding in `truthKey`:**  
Before hashing, each coordinate is rounded to the nearest grid cell:
```
rounded = round(value_cm * 10^decimal_places) / 10^decimal_places
```
This absorbs floating-point drift from movement arithmetic (e.g. `100.00001 cm` and `100.00000 cm` both map to the same cell). Rounding uses `std::round` + index multiplication, never accumulated addition.

**Startup sequence (called by `main`):**
1. `setMapBounds(bounds)` — stores the flight volume and resolution. Must come before any `setTruthCell` call so keys are rounded with the correct precision.
2. `setTruthCell(p, Occupied)` — called once per wall cell by `MapFileReader`.
3. `setDronePosition(missionConfig.initial_position)` — places the drone at its starting point.

**Important methods:**

| Method | What it does |
|---|---|
| `setDronePosition` | Overwrites drone position (called by `MovementMock` after a valid move). |
| `dronePosition` | Returns current drone position (read by `PositionMock` and `LidarMock`). |
| `setMapBounds` | Stores flight volume + resolution. Call once before `setTruthCell`. |
| `hasBounds` / `mapBounds` | Query whether bounds have been set and what they are. |
| `setTruthCell` | Records a wall cell (called by `MapFileReader` at startup). |
| `truthValue` | Returns ground truth at a position: `OutOfBounds`, `Empty`, or `Occupied`. |
| `clearGroundTruth` | Removes all stored wall cells; bounds are preserved. |

---

### `CollisionDetector.h / .cpp`

**What it is:** Checks whether a point or the drone's physical body overlaps with an `Occupied` cell.

**Constructor:**
```cpp
CollisionDetector(const SimulationState& state,
                  DroneConfig drone_cfg,
                  double step_xy_cm,
                  double step_height_cm);
```
`step_xy_cm` and `step_height_cm` come from `MissionConfig` resolution:  
`step = 10^(-decimal_places)` cm — exactly one grid cell.

**Key design decision — partial-face checks during movement:**  
Instead of re-sampling the drone's entire bounding box on every step, only the **leading face** is checked as the drone moves:
- `advance` → forward face (width × height slice at `dl = +half_length`)
- `elevate` upward → top face (length × width slice at `dh = +half_height`)
- `elevate` downward → bottom face (`dh = -half_height`)

The interior was already confirmed clear at the previous step. Full-box checks are only used for static placement validation.

**No floating-point loop accumulation:**  
All face grids are indexed by integers. Each sample offset is computed as:
```
offset(i) = -half + i * step
```
This avoids the classic `d += step` drift where the final boundary cell might be missed.

**Important methods:**

| Method | What it does |
|---|---|
| `intersectsOccupied(point)` | Single grid-cell check — is this exact cell occupied? Used by lidar ray-march. |
| `intersectsFootprint(pos)` | Full 3-D oriented bounding box check — all cells in width × length × height volume. |
| `intersectsForwardFace(pos)` | Front face only — width × height slice entering new space during `advance`. |
| `intersectsElevateFace(pos, upward)` | Top or bottom face — length × width slice entering new space during `elevate`. |

**Oriented bounding box geometry** (shared by all footprint methods via `makeGeom`):
```
world_x = cx + dl × fwd_x + dw × rgt_x
world_y = cy + dl × fwd_y + dw × rgt_y
world_z = ch + dh
```
where `fwd = (cos θ, sin θ)` and `rgt = (sin θ, -cos θ)`, and `dl/dw/dh` range over the half-dimensions from `DroneConfig`.

---

### `MovementMock.h / .cpp`

**What it is:** The simulated movement driver. Implements `IMovementDriver`.

**Constructor:**
```cpp
MovementMock(SimulationState& state,
             DroneConfig drone_cfg,
             const MissionConfig& mission_cfg);
```
Step sizes are computed once at construction: `step_xy_cm_ = 10^(-xy_decimal_places)`.  
A `CollisionDetector` is stored as a member, initialised with the same step sizes.

**What each command does:**

| Command | Limits checked | Path check | On failure |
|---|---|---|---|
| `rotate(dir, angle)` | `0 ≤ angle ≤ max_rotate_per_command` | None (rotation is instantaneous) | Log + reject |
| `advance(distance)` | `distance ≥ 0` and `≤ max_advance_per_command` | Forward face at every step + destination | Log + reject |
| `elevate(distance)` | `|distance| ≤ max_elevate_per_command` | Top/bottom face at every step + destination | Log + reject |

If any check fails the position is **not changed** and a message is written via `log::info`.

**Angle convention:**
- `xy_angle = 0°` → facing `+x` direction (east).
- Increasing angle → turning left (counterclockwise).
- `advance` uses `cos(angle)` / `sin(angle)` to project the step onto x/y.

---

### `PositionMock.h / .cpp`

**What it is:** The simulated GPS / position sensor. Implements `IPositionSensor`.

**What it does:** Returns the drone's exact position from `SimulationState` on every call — perfect, noise-free, zero delay. Nothing to implement; it is complete.

---

### `LidarBeamCalculator.h / .cpp`

**What it is:** Computes the unit-vector direction of every lidar beam for one scan, given `LidarConfig`. *(Milestone 2 — stub returns empty vector.)*

**The FOV model (to be implemented):**
- **Circle 0** — 1 central beam, direction `(1, 0, 0)` (forward).
- **Circle n** — `4^n` beams at elevation angle `θ_n = atan(n × D / Z_min)` from the forward axis, evenly spaced around the cone circumference: `φ_k = k × 360° / 4^n`.
- Total beams for `FOVC` circles: `(4^FOVC − 1) / 3`. Example: `FOVC=5` → 341 beams.

All directions are returned in the **local frame** (`+x` = forward). `LidarMock` rotates them into world space.

---

### `LidarMock.h / .cpp`

**What it is:** The simulated lidar sensor. Implements `ILidarSensor`. *(Milestone 2 — stub returns empty result.)*

**`scan(xy_offset, height_angle)` — both parameters are optional:**
- `xy_offset` — rotates the entire cone horizontally (left/right) on top of the drone's heading. Lets the algorithm scan any horizontal direction without moving the drone.
- `height_angle` — tilts the cone vertically (positive = up, negative = down). `90°` points straight up; `-90°` straight down.

Combining both gives full spherical coverage from a stationary drone.

**How a scan will work per beam (Milestone 2):**
1. Get beam unit vectors from `LidarBeamCalculator`.
2. Apply `height_angle` tilt (rotation around local `+y`).
3. Apply `drone.xy_angle + xy_offset` rotation (rotate local frame into world frame).
4. Ray-march from drone position: step at `stepCm()` intervals (= min of xy/height cell sizes).
5. At each step query `SimulationState::truthValue`. On first `Occupied` hit → record `LidarHit{azimuth, elevation, distance}` and stop.
6. Beams that reach `Z_max` without a hit produce no entry.

**Range limits:**
- Hits below `Z_min` — detected but distance is unreliable (marked accordingly).
- Hits beyond `Z_max` — not detected at all.

---

## Data flow summary

```
map_input.txt
      │
      ▼
MapFileReader ──setMapBounds()──▶ SimulationState ◀── setDronePosition() (from MissionConfig)
              ──setTruthCell()──▶       │
                                        │
                    ┌───────────────────┼───────────────────┐
                    │                   │                    │
            MovementMock          PositionMock          LidarMock
        (reads+writes pos,     (reads pos only,    (reads pos + cells,
         checks cells via       pure passthrough)   fires beams via
         CollisionDetector)                         LidarBeamCalculator)
                    │                                        │
           CollisionDetector                      LidarBeamCalculator
           (reads cells only)                     (pure geometry, no state)
                                                            │
                                                     BuildingMap::set()
                                                   (Partner B — drone's map)
                                                            │
                                                     map_output.txt → Scorer
```
