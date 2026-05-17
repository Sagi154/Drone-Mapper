# DroneMapper — two-person work plan

Simple split so you can work in parallel without stepping on the same files all day.

## Roles

**Partner A — Simulation + sensors**  
`src/simulation/*`, lidar behavior, collision/movement, anything that reads the **secret** ground-truth map.

**Partner B — Mission + drone map + glue**  
`src/config/*`, `src/io/*`, `src/mapping/*`, `src/algorithm/*`, `src/main.cpp`, file formats, scoring, wiring the main flow.

**Shared**  
`include/` — treat as the **contract**. Agree on signatures early; only one person should touch it per day if you can avoid merge pain.

---

## Milestone 1 — Inputs and world (2–3 days)

**A**

- Ground truth storage in `SimulationState` (whatever structure you pick).
- `MovementMock` + `CollisionDetector` so invalid moves fail clearly (per assignment).
- `PositionMock` stays thin: reads state.

**B**

- Decide **real** formats for `drone_config.txt`, `mission_config.txt`, `map_input.txt` / `map_output.txt`.
- Parsers + `ErrorLogger` / `input_errors.txt` rules.
- `MapFileReader` loads truth into `SimulationState` (expose a small API A and B agree on).

**Done when:** `main` loads configs + map, builds state, no crashes on sane files.

---

## Milestone 2 — Sensing and map (3–5 days)

**A**

- `LidarBeamCalculator` + `LidarMock::scan()` consistent with the assignment lidar model (iterate: range + hits first, then FOV/circles/blind spots).

**B**

- `BuildingMap`: bounds (`-2`), unknown (`-1`), quantization from mission resolution.
- `MapFileWriter` matches your map format.

**Done when:** a tiny hand-made map + a few moves/scans updates the drone map in a believable way.

---

## Milestone 3 — Autonomy (largest chunk)

**B (lead)**

- `DroneAlgorithm`: deterministic exploration, when to scan, when to report finished.
- `ExplorationFrontier` / `BlindSpotAnalyzer` only as much code as you **actually** need.

**A (support)**

- Edge cases from real runs: collision sizing, lidar pose, “too close” distance `0`, first-hit-only along a beam.

**Done when:** a simple box-shaped map completes end-to-end on your machine.

---

## Milestone 3 — Algorithm design rationale

### What it does

The drone explores the building autonomously using a frontier-based BFS loop.
A **frontier** is a confirmed-empty cell adjacent to at least one `NotMapped`
cell — moving there and scanning may uncover new space.

### Three-phase state machine (`DroneAlgorithm::tick()`)

Each call to `tick()` advances exactly one phase:

| Phase | What happens |
|-------|-------------|
| **Scanning** | `fullScan()` fires a full spherical lidar sweep (elevation −90°→+90°, 360° azimuth at each tier). All hits are fused into the drone's `IBuildingMap` via `applyLidarHitsToMap`. Transitions to Planning. |
| **Planning** | `ExplorationFrontier::findPath()` runs BFS through sphere-safe cells to find the nearest frontier. If found, the path is stored and the phase transitions to Moving. If none exists, `finished_` is set. |
| **Moving** | `executeNextStep()` issues one movement command (elevate, rotate, or advance) toward the next waypoint. Once the waypoint is reached the index advances; on path completion the phase returns to Scanning. |

### Key design decisions

**Sphere-safe passability:** A BFS cell is only passable if every grid cell
within `min_passable_radius` of its centre is `Empty` in the drone's own map.
`NotMapped` blocks movement — it means no information, not "probably empty."

**Dense spherical scan before planning:** The lidar is a directional cone;
one scan marks only cells along beam ray paths. The angular step is
`atan(cell_size / z_min)` so no grid cell falls in a beam gap at minimum
range. Azimuth step widens near poles (`el_step / cos(el)`) for uniform
physical spacing.

**Determinism:** Scan order, BFS neighbour order (+X, −X, +Y, −Y, +Height,
−Height), and path reconstruction are all fixed. No randomness.

### Interfaces other modules should use

- `DroneAlgorithm(lidar, pos, move, map, DroneConfig)` — main entry point.
- `ExplorationFrontier::findPath(map, start, drone_radius)` — returns
  `PathResult{found, path}`; usable independently for testing.
- `BlindSpotAnalyzer` — empty stub; blind spots become frontiers naturally.

---

## Milestone 4 — Score, hardening, submission (2–4 days)

**B**

- `Scorer` + printed score; tune so 100 is reachable where the drone can see everything relevant.
- `main`: paths, required filenames, **no `exit()`** — assignment rules.

**A**

- Weird maps: tight corridors, height changes, lidar limits.

**Together**

- `tests/`: A owns simulation/lidar tests; B owns parser/map/scorer tests.
- `docs/HLD.pdf` + `docs/diagrams/*.puml` — split: one does diagrams, one writes design/testing text.

**Optional last:** `visualizer/` — whoever has bandwidth.

---

## Working rules

1. One branch per milestone; merge at each “Done when”.
2. Coordinate `include/` changes (chat before you both edit the same header).
3. Agree early what **occupied vs empty** means in truth vs in the drone map.
4. Interface mismatch: **15-minute call**, change one function, move on.

---

## Expectations

- A usually carries the longest **debug tail** (lidar + collision).
- B usually carries the second-longest chunk (**algorithm + scoring**).

If one finishes early, they pick up tests, HLD polish, or the visualizer.
