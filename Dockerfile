# CI image: Ubuntu 24.04 C++ devcontainer stack + vcpkg (same family as the course docker_example).
# Reproduces Linux configure / build / test in one place for GitHub Actions and local checks.
#
# Usage (from repo root):
#   docker build -t drone-mapper-ci .
#
# The base image sets VCPKG_ROOT (typically /usr/local/vcpkg). Dependencies come from vcpkg.json
# during configure. The SFML visualizer is disabled here (headless CI; smaller context).
FROM mcr.microsoft.com/devcontainers/cpp:2-ubuntu24.04

WORKDIR /workspace

# Copy only what the CMake project needs (keeps build context small; see .dockerignore).
COPY vcpkg.json CMakeLists.txt CMakePresets.json ./
COPY include ./include
COPY src ./src
COPY tests ./tests

ARG CMAKE_BUILD_PARALLEL_LEVEL=4
ENV CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL}

# Explicit toolchain path so CI does not depend on shell preset wiring inside the container.
RUN cmake -G Ninja -S . -B build/ci \
        -DCMAKE_BUILD_TYPE=Release \
        -DDRONE_MAPPER_BUILD_VISUALIZER=OFF \
        -DCMAKE_TOOLCHAIN_FILE="${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" \
    && cmake --build build/ci \
    && ctest --test-dir build/ci --output-on-failure
