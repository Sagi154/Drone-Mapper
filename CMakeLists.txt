cmake_minimum_required(VERSION 3.21)
project(DroneMapper LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(mp-units CONFIG REQUIRED)

file(GLOB_RECURSE DRONE_MAPPER_LIB_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
list(FILTER DRONE_MAPPER_LIB_SOURCES EXCLUDE REGEX "/main\\.cpp$")

add_library(drone_mapper_lib STATIC ${DRONE_MAPPER_LIB_SOURCES})
target_include_directories(drone_mapper_lib
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_compile_features(drone_mapper_lib PUBLIC cxx_std_20)
target_link_libraries(drone_mapper_lib PUBLIC mp-units::mp-units)

add_executable(drone_mapper src/main.cpp)
target_include_directories(drone_mapper PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(drone_mapper PRIVATE drone_mapper_lib)

find_package(GTest CONFIG REQUIRED)
enable_testing()
add_subdirectory(tests)

option(DRONE_MAPPER_BUILD_VISUALIZER "Build SFML drone_visualizer target" ON)
if(DRONE_MAPPER_BUILD_VISUALIZER)
  find_package(SFML CONFIG REQUIRED COMPONENTS Graphics Window System)
  add_subdirectory(visualizer)
endif()
