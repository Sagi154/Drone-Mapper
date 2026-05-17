# Frontier-Based BFS Exploration with BFS Path Planning

## Overview

The drone maintains an internal 3D grid map of the building, where each cell is marked as one of the following:

```text
UNKNOWN
EMPTY
OCCUPIED
OUTSIDE_BOUNDARIES
```

The drone never uses the real input map directly. The real map is used only by the mock lidar sensor. The drone updates its own discovered map only according to lidar scan results and its own movement/position information.

## Main Idea

The algorithm is based on frontier exploration.

A **frontier** is a known-empty cell that is adjacent to at least one unknown cell. Such a cell is useful because moving to it and scanning from there may reveal more of the map.

The drone repeatedly:

```text
1. Scans from its current physical position.
2. Updates its internal discovered map.
3. Finds the nearest reachable frontier cell.
4. Plans a safe path to that frontier using BFS.
5. Moves along the path using rotate / advance / elevate commands.
6. Repeats until no reachable frontier remains.
```

## Map Updates from Lidar

After each scan:

```text
Cells crossed by lidar beams are marked EMPTY.
Cells hit by lidar beams are marked OCCUPIED.
Cells that were not reached by any beam remain UNKNOWN.
Cells outside the mission boundaries are marked OUTSIDE_BOUNDARIES.
```

The algorithm does not interpolate blind spots. A cell is considered mapped only if it was actively observed by a lidar beam.

## BFS Usage

The BFS is run from the drone’s current physical position.

It expands only through cells that are already known to be empty. This means the drone never plans a path through unknown or occupied cells.

The first frontier found by this BFS is the nearest reachable frontier.

The BFS also stores parent pointers, which are used to reconstruct the path from the drone’s current position to the selected frontier.

## Movement

After BFS finds the nearest reachable frontier, the drone reconstructs the path and physically follows it.

The drone does not “jump” to BFS nodes. BFS only decides where the drone should go and how to get there safely.

The actual movement is performed using the movement driver commands:

```text
Rotate
Advance
Elevate
```

## Safety Rule

The key safety rule is:

```text
The drone may scan unknown space, but it may only move through confirmed empty space.
```

This helps ensure that the algorithm never causes the drone to collide with obstacles.

## Determinism

The algorithm is deterministic because:

```text
The scan pattern is fixed.
The BFS neighbor order is fixed.
Tie-breaking between equal-distance frontiers is fixed.
No randomness is used.
```

For example, the neighbor order can always be:

```text
+X, -X, +Y, -Y, +Height, -Height
```

## Stopping Condition

The algorithm finishes when there are no reachable frontier cells left.

At that point, every reachable area that can be mapped safely has been explored, and unreachable or unobserved cells may remain unknown.
