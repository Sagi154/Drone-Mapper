# Exploration Frontier — `ExplorationFrontier::findPath()`

## Purpose

After scanning, the drone needs to decide where to go next. `findPath()`
answers: "what is the nearest cell I can safely reach that might reveal new
space if I scan from there?"

## Definitions

**Passable cell:** A grid cell whose centre is `Empty` AND every grid cell
within `drone_radius` of that centre is also `Empty` in the drone's map.
`NotMapped` within the sphere blocks movement — it means no information, not
"safe to fly through."

**Frontier cell:** A passable cell that has at least one axis-aligned neighbour
(+X, −X, +Y, −Y, +Height, −Height) that is **not** sphere-passable. That
neighbour may be `NotMapped`, `Occupied`, or `OutOfBounds` — the drone can
stand at the frontier but cannot safely move its centre onto that neighbour.
Moving to a frontier and scanning may reveal space beyond the blocked neighbour.
The start cell is never treated as the goal; BFS only stops on frontiers reached
after at least one grid step.

## Algorithm

BFS from the drone's current position through passable cells:

```
1. Quantize start position to the map grid.
2. Check that start itself is passable; return found=false if not.
3. Enqueue start. Mark it visited via parent_of[start] = start.
4. While queue is not empty:
     a. Dequeue current cell.
     b. If current is a frontier → reconstruct path, return found=true.
     c. For each axis-aligned neighbour (+X,-X,+Y,-Y,+H,-H):
          - Skip if already visited.
          - Skip if not passable (isSpherePassable check).
          - Otherwise enqueue and record parent.
5. If queue empties with no frontier found → return found=false.
```

BFS guarantees the **first** frontier found is the nearest one (fewest grid
steps from start).

## Sphere-safe passability check (`isSpherePassable`)

For a candidate cell centre `(cx, cy, ch)`:

```
rx = ceil(drone_radius / xy_step)
rh = ceil(drone_radius / h_step)

for dx in [-rx .. rx]:
  for dy in [-rx .. rx]:
    for dz in [-rh .. rh]:
      if dx² * xy_step² + dy² * xy_step² + dz² * h_step² > radius²:
        continue   // outside sphere
      if map.get(centre + offset) != Empty:
        return false
return true
```

Every grid cell whose centre actually lies within the sphere must be `Empty`.
`OutOfBounds`, `Occupied`, and `NotMapped` all fail the check.

## Path reconstruction

Parent pointers stored in `parent_of` are followed from the frontier back to
start. The resulting list is reversed so it runs start → … → frontier.
Start is excluded from the path (the drone is already there); the frontier
cell is included as the last waypoint.

## Determinism

Neighbour expansion order is always `+X, −X, +Y, −Y, +Height, −Height`.
No randomness. Given the same map state and start position, the same path is
always returned.

## Stopping condition

`found=false` is returned when the BFS queue empties with no frontier reached.
This means either:
- Every reachable area is fully confirmed `Empty` (no `NotMapped` neighbours).
- The drone's starting sphere neighbourhood itself is not fully `Empty`, making
  all navigation impossible from this position.

## Relevant source

- `src/algorithm/ExplorationFrontier.h` — `PathResult` struct, `findPath()` declaration
- `src/algorithm/ExplorationFrontier.cpp` — `isSpherePassable`, `isFrontier`, BFS
- `src/mapping/SparseKey.h/.cpp` — grid quantization used for BFS keys
- `include/mapping/IBuildingMap.h` — `get()` queried for passability checks
