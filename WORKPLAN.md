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

## Milestone 4 — Score, hardening, submission (2–4 days)

**B**

- `Scorer` + printed score; tune so 100 is reachable where the drone can see everything relevant.
- `main`: paths, required filenames, **no `exit()`** — assignment rules.

**A**

- Weird maps: tight corridors, height changes, lidar limits.
- **Done** — 7 edge-case maps generated in `test_data/maps/` (see `docs/test_maps.md`):
  1. `map_open_room` — empty baseline
  2. `map_narrow_corridor` — 40 cm tunnel (passable)
  3. `map_impassable_gap` — 20 cm gap (impassable, < 30 cm min)
  4. `map_enclosed_room` — sealed box, interior permanently inaccessible
  5. `map_pillars` — 10 column obstacles with lidar shadow zones
  6. `map_dead_end` — U-shaped hall with one dead-end arm
  7. `map_low_obstacles` — walls only z=0..40; drone can fly over at z=60
- Generator script: `scripts/generate_test_maps.py` (re-run to regenerate)

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
