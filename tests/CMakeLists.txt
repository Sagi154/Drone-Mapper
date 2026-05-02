add_executable(
  drone_mapper_tests
  test_lidar_mock.cpp
  test_building_map.cpp
  test_drone_algorithm.cpp
  test_config_parser.cpp
  test_scorer.cpp
  test_collision_detector.cpp
)

target_include_directories(drone_mapper_tests PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_link_libraries(drone_mapper_tests PRIVATE drone_mapper_lib GTest::gtest
                                                  GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(drone_mapper_tests)
