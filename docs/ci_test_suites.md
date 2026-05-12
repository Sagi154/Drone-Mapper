# CI Test Suites

## What this change does

The CI Docker workflow now separates building from testing:

- `Dockerfile` configures and builds the Linux/vcpkg project.
- `.github/workflows/ci.yml` runs the compiled tests in separate `ctest` steps.
- `tests/CMakeLists.txt` assigns labels to the existing GoogleTest suites so they can be run independently.

This keeps tests from running twice while making failures easier to identify in GitHub Actions.

## Key design decisions

- **Build-only Docker image:** `docker build` stops after `cmake --build build/ci`.
  This keeps image creation focused on configure/build failures.
- **Runtime test execution:** CI invokes `ctest` with `docker run`.
  This makes test failures appear in named workflow steps instead of inside the Docker build log.
- **CTest labels by module:** each test suite has a label that matches the code area it exercises.
  Labels are assigned through `gtest_discover_tests(... PROPERTIES LABELS ...)`.
- **Separate test executables:** each labeled suite is its own executable.
  This gives CTest a clean way to partition suites while still sharing the same library and GTest setup.

## Test suite labels

| Label | Test files |
|---|---|
| `config` | `test_config_parser.cpp` |
| `mapping` | `test_building_map.cpp`, `test_scorer.cpp` |
| `algorithm` | `test_drone_algorithm.cpp` |
| `simulation` | `test_collision_detector.cpp`, `test_lidar_beam_calculator.cpp`, `test_lidar_mock.cpp`, `test_movement_mock.cpp`, `test_position_mock.cpp`, `test_simulation_state.cpp` |
| `integration` | `test_integration.cpp` |

## Interfaces to use

Run every test suite locally:

```sh
docker build -t drone-mapper-ci .
docker run --rm drone-mapper-ci ctest --test-dir build/ci --output-on-failure
```

Run one suite by label:

```sh
docker run --rm drone-mapper-ci ctest --test-dir build/ci -L simulation --output-on-failure
```

Available labels:

```sh
config
mapping
algorithm
simulation
integration
```

The GitHub Actions workflow runs the same label commands as separate steps after the image is built.

## Adding a new test suite

Add new test files to the appropriate call in `tests/CMakeLists.txt`:

```cmake
add_drone_mapper_test_suite(drone_mapper_simulation_tests simulation
  test_collision_detector.cpp
  test_new_simulation_case.cpp
)
```

If a new module needs its own CI step, create a new `add_drone_mapper_test_suite(...)` entry with a new label, then add a matching `ctest -L <label>` step in `.github/workflows/ci.yml`.
