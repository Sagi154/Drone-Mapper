# DroneMapper

Repository layout matches the course structure plan: `include/` contracts, `src/` modules, `tests/`, `visualizer/`, `data/`, `docs/`.

## Build (vcpkg)

Set `VCPKG_ROOT` and configure with the presets in `CMakePresets.json`, or pass:

`cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake`

Then `cmake --build build`.

The SFML visualizer is **off** by default (no SFML in vcpkg). To build `drone_visualizer`, add `-DDRONE_MAPPER_BUILD_VISUALIZER=ON` to the configure line (vcpkg will then install SFML via the `visualizer` manifest feature).

## CI (Docker)

Linux CI uses the repo-root `Dockerfile` (Ubuntu 24.04 devcontainers C++ image + vcpkg manifest). GitHub Actions runs `docker build`, which configures with the vcpkg toolchain, builds the library, `drone_mapper`, and tests (not the SFML visualizer), and runs `ctest`.

Locally (Docker Desktop or Linux):

`docker build -t drone-mapper-ci .`

## Run

`drone_mapper [<input_output_files_path>]`

Behavior is intentionally left to your implementation; this tree is a **scaffold** (compiling stubs and architectural seams).
