# Drone Algorithm — `DroneAlgorithm`

## Overview

`DroneAlgorithm` is the top-level autonomous exploration controller. It drives
the drone through a building using a frontier-based BFS strategy: scan the
surroundings, find the nearest reachable unexplored cell, navigate to it, and
repeat until no reachable frontiers remain.

The algorithm is deterministic — given the same map and starting conditions it
always produces the same sequence of movements.

## State machine

Each call to `tick()` advances exactly **one phase**:

```
          ┌─────────────────────────────────────────────┐
          │                                             │
     ┌────▼─────┐    always    ┌──────────┐  path found ┌────────┐
─────► Scanning ├─────────────► Planning ├────────────► Moving  │
     └──────────┘              └──────────┘             └────┬───┘
                                    │                        │
                              no frontier               waypoint reached,
                                    │                   path complete
                               ┌────▼────┐                   │
                               │  Done   │◄──────────────────┘
                               │finished_│
                               └─────────┘
```

### Scanning

Calls `fullScan()`, which fires the lidar in a full spherical sweep (see
[`algorithm_scan_logic.md`](algorithm_scan_logic.md)) and fuses all results
into the drone's `IBuildingMap`. After scanning, transitions immediately to
Planning.

### Planning

Calls `ExplorationFrontier::findPath(map, current_position, min_passable_radius)`
(see [`exploration_frontier.md`](exploration_frontier.md)).

- **Path found:** stores the waypoint list in `current_path_`, resets
  `path_index_` to 0, transitions to Moving.
- **No frontier reachable:** sets `finished_ = true`. Exploration ends.

### Moving

Calls `executeNextStep()` to issue one movement command toward
`current_path_[path_index_]`, then checks `reachedWaypoint()`.

When a waypoint is reached:
- `path_index_` is incremented.
- If all waypoints are consumed, transitions back to Scanning (the drone is
  now at the frontier, ready to scan from there).

## `executeNextStep()` — movement priority

Because BFS waypoints are axis-aligned (each differs in at most one axis),
movement to a waypoint takes at most two ticks:

| Priority | Condition | Command |
|----------|-----------|---------|
| 1st | Height of target differs from current height | `elevate(delta)` clamped to `max_elevate_per_command` |
| 2nd | Drone is not facing the target | `rotate(shortest direction, delta)` clamped to `max_rotate_per_command` |
| 3rd | Drone is facing target at same height | `advance(max_advance_per_command)` |

Only one command is issued per `tick()` in Moving phase.

## `reachedWaypoint()` — arrival check

A waypoint is considered reached when the drone's position is within half a
grid step of the waypoint centre in every axis:

```
|dx| ≤ xy_step / 2
|dy| ≤ xy_step / 2
|dh| ≤  h_step / 2
```

## Stopping condition

`isFinished()` returns true when `ExplorationFrontier::findPath()` returns
`found=false` — no sphere-safe passable cell adjacent to unknown space is
reachable from the drone's current position.

## Constructor

```cpp
DroneAlgorithm(ILidarSensor& lidar,
               IPositionSensor& pos,
               IMovementDriver& move,
               IBuildingMap& map,
               const DroneConfig& cfg);
```

`DroneConfig` provides all capability parameters: `min_passable_radius`,
`max_advance_per_command`, `max_rotate_per_command`, `max_elevate_per_command`,
and the lidar configuration (`z_min`, `z_max`, `fov_circles`, `circle_spacing_d`).

## Relevant source

- `src/algorithm/DroneAlgorithm.h` — class declaration, Phase enum, private fields
- `src/algorithm/DroneAlgorithm.cpp` — `tick()`, `fullScan()`, `executeNextStep()`
- `src/algorithm/ExplorationFrontier.h/.cpp` — BFS frontier search
- `include/config/DroneConfig.h` — capability parameters
- `tests/test_drone_algorithm.cpp` — unit and end-to-end tests
- `tests/test_exploration_frontier.cpp` — BFS passability and path tests
