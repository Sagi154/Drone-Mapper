add_executable(
  drone_visualizer
  main_visualizer.cpp
  MapRenderer.cpp
  CameraController.cpp
)

target_include_directories(drone_visualizer PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(drone_visualizer PRIVATE SFML::Graphics SFML::Window SFML::System)
