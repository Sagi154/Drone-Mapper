include(GoogleTest)

function(add_drone_mapper_test_suite target_name label)
    add_executable(${target_name} ${ARGN})
    target_include_directories(${target_name} PRIVATE "${CMAKE_SOURCE_DIR}/src")
    target_link_libraries(${target_name} PRIVATE drone_mapper_lib GTest::gtest
            GTest::gtest_main)
    gtest_discover_tests(${target_name} PROPERTIES LABELS "${label}")
endfunction()

add_drone_mapper_test_suite(drone_mapper_config_tests config
        test_config_parser.cpp
)

add_drone_mapper_test_suite(drone_mapper_mapping_tests mapping
        test_building_map.cpp
        test_scorer.cpp
)

add_drone_mapper_test_suite(drone_mapper_algorithm_tests algorithm
        test_drone_algorithm.cpp
)

add_drone_mapper_test_suite(drone_mapper_simulation_tests simulation
        test_collision_detector.cpp
        test_lidar_beam_calculator.cpp
        test_lidar_mock.cpp
        test_movement_mock.cpp
        test_position_mock.cpp
        test_simulation_state.cpp
)

add_drone_mapper_test_suite(drone_mapper_integration_tests integration
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
