# DroneMapper

Repository layout matches the course structure plan: `include/` contracts, `src/` modules, `tests/`, `visualizer/`, `data/`, `docs/`.

## Build (vcpkg)

Set `VCPKG_ROOT` and configure with the presets in `CMakePresets.json`, or pass:

`cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake`

Then `cmake --build build`.

## Run

`drone_mapper [<input_output_files_path>]`

Behavior is intentionally left to your implementation; this tree is a **scaffold** (compiling stubs and architectural seams).
