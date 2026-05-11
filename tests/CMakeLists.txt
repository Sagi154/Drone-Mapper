add_executable(
  drone_mapper_tests
  test_lidar_mock.cpp
  test_building_map.cpp
  test_drone_algorithm.cpp
  test_config_parser.cpp
  test_scorer.cpp
  test_collision_detector.cpp
  test_simulation_state.cpp
  test_movement_mock.cpp
  test_position_mock.cpp
  test_lidar_beam_calculator.cpp
  test_integration.cpp
)

target_include_directories(drone_mapper_tests PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(drone_mapper_tests PRIVATE drone_mapper_lib GTest::gtest
                                                  GTest::gtest_main)

# Path to committed sample inputs used by end-to-end parser tests.
target_compile_definitions(
  drone_mapper_tests PRIVATE "TEST_DATA_DIR=\"${CMAKE_SOURCE_DIR}/test_data\"")

include(GoogleTest)
gtest_discover_tests(drone_mapper_tests)
