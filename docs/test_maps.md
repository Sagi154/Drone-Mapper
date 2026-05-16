# Test Maps

All maps live in `test_data/maps/`.  Each map has a paired `mission_*.txt`
configuration file in the same directory.  Maps are generated (or regenerated)
by running:

```bash
python scripts/generate_test_maps.py
```

The generator script is fully documented and can be re-run any time to reset
or extend the map set.

---

## Map file format

```
# comment
bounds min_x max_x min_y max_y min_z max_z xy_dp height_dp
occupied x y z
```

All coordinates are **centimetres**.  `dp=0` gives 1 cm cells.
Unspecified in-bounds cells default to **Empty** in `SimulationState`.
See `docs/building_map_and_output.md` for the full grammar.

---

## Drone constraints (reference)

| Parameter | Value |
|---|---|
| `min_passable_width` | 30 cm |
| `min_passable_length` | 30 cm |
| `min_passable_height` | 20 cm |
| `lidar_z_min` | 20 cm (above drone) |
| `lidar_z_max` | 120 cm (above drone) |
| Typical flight height | 50 cm |

---

## Map catalogue

### 1. `map_open_room.txt` — Open room (baseline)

- **Space:** 500 x 500 x 200 cm
- **Obstacles:** none
- **Purpose:** Sanity check.  Drone should fully explore the space with no
  collisions.  Expected final score = 100 % mapped.
- **Edge case tested:** No obstacle path; scorer receives a fully-empty truth map.

---

### 2. `map_narrow_corridor.txt` — Narrow corridor (passable)

- **Space:** 600 x 200 x 200 cm
- **Obstacles:** Two full-length walls at `y=80` and `y=121`
- **Corridor:** `y=81..120` — **40 cm wide** (just above the 30 cm minimum)
- **Purpose:** Tests that the drone can navigate a tight passage without
  clipping the walls.  Lidar sees both walls simultaneously.
- **Edge case tested:** Gap width exactly 10 cm above the minimum; any
  lateral drift causes a collision.

```
y=0
 |   [south wall y=80]
 |   ─────────────────────────────────  40 cm corridor
 |   [north wall y=121]
y=200
     x=0 ─────────────────────────── x=600
```

---

### 3. `map_impassable_gap.txt` — Impassable gap

- **Space:** 500 x 500 x 200 cm
- **Obstacles:** Wall at `y=100`, `x=0..239` and `x=260..500`;
  gap at `x=240..259` = **20 cm wide** (< 30 cm minimum)
- **Purpose:** Verifies the algorithm recognises an uncrossable boundary
  and stops trying to explore the northern half.
- **Edge case tested:** Area `y=101..500` is permanently inaccessible.
  The scout should correctly classify it as NotMapped and not loop.

---

### 4. `map_enclosed_room.txt` — Enclosed inaccessible room

- **Space:** 500 x 500 x 200 cm
- **Obstacles:** 1 cm-thick perimeter walls forming a sealed 100 x 100 cm
  box at `x=200..300, y=200..300`
- **Interior:** `x=201..299, y=201..299` — **permanently inaccessible**
- **Purpose:** Tests that the scorer and algorithm handle cells the drone
  can never observe.  Interior cells must remain `NotMapped` in the output.
- **Edge case tested:** Lidar cannot see through walls; the drone has no
  entrance.  The mapping loop must terminate without trying to enter.

---

### 5. `map_pillars.txt` — Pillar maze

- **Space:** 500 x 500 x 200 cm
- **Obstacles:** 10 square pillars (10 x 10 cm each) at positions:
  `(100,100)`, `(100,400)`, `(250,150)`, `(250,350)`, `(400,100)`,
  `(400,400)`, `(175,250)`, `(325,250)`, `(250,50)`, `(250,450)`
- **Min gap between any two pillars:** ~50 cm (all passable)
- **Purpose:** Tests navigation around multiple isolated obstacles and
  correct handling of lidar **shadow zones** (areas blocked from direct LOS).
- **Edge case tested:** A pillar at `(250,50)` is near the south boundary,
  creating a very tight lane between the pillar and the map edge (~45 cm).

---

### 6. `map_dead_end.txt` — Dead-end corridor

- **Space:** 500 x 500 x 200 cm
- **Structure:** U-shaped hallway
  - Outer west wall: `x=50, y=200..400`
  - Outer east wall: `x=450, y=200..400`
  - Outer top wall: `y=400, x=50..450`
  - Inner left divider: `x=200, y=200..350`
  - Inner right divider: `x=300, y=200..350`
  - Pocket floor: `y=200, x=200..300`
- **Left arm** `(x=51..199, y=201..399)`: **dead end**, sealed at the top
- **Right arm** `(x=301..449, y=201..399)`: connects around `y=351..399`
- **Purpose:** Tests backtracking.  The drone enters the left arm, finds no
  exit, must reverse out, then navigate the right arm.
- **Edge case tested:** Dead-end detection; the algorithm must not loop
  indefinitely inside the sealed pocket.

```
   x: 50  200 300 450
       |   |   |   |
y=400  +───+   +───+   (top wall)
       |   |   |   |
y=350  |   +   +   |   (inner dividers stop here → top of U open)
       |           |
y=200  |   +───+   |   (pocket floor — seals left dead-end bottom)
       |   |   |   |
y=200  +   +   +   +   (entry — open south)
```

---

### 7. `map_low_obstacles.txt` — Low obstacles (fly-over challenge)

- **Space:** 500 x 500 x 200 cm
- **Obstacles:** Three staggered walls, **z=0..40 only**:
  - `y=150`, `x=0..350`
  - `y=300`, `x=150..500`
  - `y=450`, `x=0..350`
- **Purpose:** Tests height-aware navigation.  At `z=60` the drone clears
  all walls.  At `z=50` the footprint clips the `z=40` top edge.
- **Edge case tested:**
  - Lidar from `z=50` (absolute `z=30..170`) can still detect the `z=40`
    obstacle top — algorithm should not mark it as empty above the wall.
  - Drone at `z=50` has footprint `z=40..60` which touches the wall top.
  - Drone elevated to `z=60` passes completely over all obstacles.

---

## How to use these maps in tests

**Programmatic (in-code):**
```cpp
SimulationState state;
dmap::loadGroundTruthMap("test_data/maps/map_narrow_corridor.txt", state, logger);
MissionConfig mission = loadMissionConfig("test_data/maps/mission_narrow_corridor.txt");
```

**As a full pipeline run** (replace the default `data/` inputs):
```bash
./DroneMapper \
    test_data/maps/map_narrow_corridor.txt \
    test_data/maps/mission_narrow_corridor.txt \
    test_data/drone_config.txt
```

---

## Regenerating maps

Edit `scripts/generate_test_maps.py` and re-run:

```bash
python scripts/generate_test_maps.py
```

The script validates that all generated files match the map grammar used by
`MapFileReader::loadGroundTruthMap`.
