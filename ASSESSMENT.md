# Architecture & Workplan Assessment — DroneMapper (Assignment 1)

> Based on: `Advanced Topics TAU 2026B - Assignment 1 - v2.pdf` and current codebase structure.  
> Deadline: **May 17, 2026**

---

## Verdict: Mostly just right, with a few genuinely overkill pieces

---

## What the assignment actually requires

A single-binary C++ simulator that:

- Reads 3 text files (`drone_config.txt`, `mission_config.txt`, `map_input.txt`)
- Runs a mock-based simulation loop (lidar, position, movement — all mocks)
- Drone algorithm explores using BFS/DFS — **not required to be smart**
- Writes `map_output.txt`, prints a score 0–100
- No crash, no `exit()`, error recovery → `input_errors.txt`
- HLD PDF with UML diagrams

Bonuses: GTest, logging, visualizer.

---

## WORKPLAN.md vs the assignment

The WORKPLAN is **well-calibrated for the most part.** The milestone breakdown
(inputs → sensing → autonomy → scoring/hardening) maps cleanly to the actual deliverables,
and the A/B split is sensible. The "done when" gates are realistic.

---

## Architecture pieces that are just right

| Component | Why it fits |
|---|---|
| `SimulationState`, `CollisionDetector`, `MovementMock`, `PositionMock`, `LidarMock`, `LidarBeamCalculator` | Directly required — the assignment explicitly asks for mock sensors and an engine mock |
| `BuildingMap` (sparse 3D matrix, values 0/1/-1/-2) | Exact API the assignment specifies |
| `DroneAlgorithm` | Required |
| `Scorer` | Required |
| `MapFileReader` / `MapFileWriter` / `ErrorLogger` | Required |
| `DroneConfigParser` / `MissionConfigParser` | Required |
| `ILidarSensor`, `IPositionSensor`, `IMovementDriver` interfaces | Required — assignment says "drone is not aware it's a mock" |
| GTest suite | Bonus-worthy, already budgeted in Milestone 4 |
| `visualizer/` | Bonus-worthy, marked optional |

---

## Pieces that are overkill (or at risk of becoming so)

### 1. `BlindSpotAnalyzer`
The assignment does require handling blind spots, but a standalone class is heavy for Ex1.
A well-written algorithm loop that simply gets closer or re-scans when a gap is detected
is sufficient. This close to the deadline (May 17), a bloated `BlindSpotAnalyzer` is a
time sink. Keep it thin, or inline the logic into `DroneAlgorithm`.

### 2. `ExplorationFrontier`
The workplan itself warns: *"only as much code as you actually need."* BFS/DFS with a
simple visited set is all Ex1 requires. A full frontier class is fine if it stays thin;
risky if it grows into something complex.

### 3. `CameraController` in the visualizer
The visualizer is a legitimate bonus target. A 3D camera controller is scope creep.
The assignment explicitly says the visualizer can be "based on the input and output files
as an external utility." A simple 2D slice viewer per height level is faster to build
and scores the bonus just as well.

### 4. `Logger` (full infrastructure)
The assignment only needs:
- `input_errors.txt` for recoverable parse errors
- A screen print for unrecoverable errors

A general-purpose logger is more than required. If it's already built and cheap to maintain,
keep it. If it's not done yet, skip it — the bonus description says "adding logging" counts,
so even a thin file-based logger qualifies.

### 5. Duplicate file paths (cosmetic, but check it)
The file tree shows both `src/simulation/CollisionDetector.h` and
`src\simulation\CollisionDetector.h` (forward vs backslash). This is likely a Windows
path artifact, but confirm you're not accidentally maintaining two diverging copies of
the same file.

---

## Bottom line

The WORKPLAN.md is **well-structured and realistic** for a two-person team. It doesn't
promise anything the assignment doesn't ask for, and it correctly marks optional items
as optional.

**The risk is not the plan — it's implementation scope creep** in `BlindSpotAnalyzer`
and `ExplorationFrontier` this close to May 17. Keep those classes thin or collapse
them into `DroneAlgorithm` if they start growing beyond a single responsibility.
