cmake_minimum_required(VERSION 3.20)
project(Scandium VERSION 0.5.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SCANDIUM_BUILD_TESTS "Build Scandium tests" ON)

add_library(scandium_core
    src/math/vec3.cpp
)
target_include_directories(scandium_core PUBLIC include)
target_compile_features(scandium_core PUBLIC cxx_std_20)

if(MSVC)
    target_compile_options(scandium_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(scandium_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(scandium_demo src/main.cpp)
target_link_libraries(scandium_demo PRIVATE scandium_core)

add_executable(scandium_ascii_demo src/simulation_demo.cpp)
target_link_libraries(scandium_ascii_demo PRIVATE scandium_core)

add_executable(scandium_arena_demo src/arena_demo.cpp)
target_link_libraries(scandium_arena_demo PRIVATE scandium_core)

add_executable(scandium_render_demo src/render_demo.cpp)
target_link_libraries(scandium_render_demo PRIVATE scandium_core)

add_executable(scandium_collision_benchmark benchmarks/collision_baseline.cpp)
target_link_libraries(scandium_collision_benchmark PRIVATE scandium_core)

add_executable(scandium_collision_comparison benchmarks/collision_comparison.cpp)
target_link_libraries(scandium_collision_comparison PRIVATE scandium_core)

add_executable(scandium_navigation_comparison benchmarks/navigation_comparison.cpp)
target_link_libraries(scandium_navigation_comparison PRIVATE scandium_core)

add_executable(scandium_simulation_scale benchmarks/simulation_scale.cpp)
target_link_libraries(scandium_simulation_scale PRIVATE scandium_core)

if(SCANDIUM_BUILD_TESTS)
    enable_testing()

    add_executable(scandium_tests tests/vec3_tests.cpp)
    target_link_libraries(scandium_tests PRIVATE scandium_core)
    add_test(NAME scandium_vec3_tests COMMAND scandium_tests)

    add_executable(scandium_mat4_tests tests/mat4_tests.cpp)
    target_link_libraries(scandium_mat4_tests PRIVATE scandium_core)
    add_test(NAME scandium_mat4_tests COMMAND scandium_mat4_tests)

    add_executable(scandium_aabb_tests tests/aabb_tests.cpp)
    target_link_libraries(scandium_aabb_tests PRIVATE scandium_core)
    add_test(NAME scandium_aabb_tests COMMAND scandium_aabb_tests)

    add_executable(scandium_physics_tests tests/physics_tests.cpp)
    target_link_libraries(scandium_physics_tests PRIVATE scandium_core)
    add_test(NAME scandium_physics_tests COMMAND scandium_physics_tests)

    add_executable(scandium_astar_tests tests/astar_tests.cpp)
    target_link_libraries(scandium_astar_tests PRIVATE scandium_core)
    add_test(NAME scandium_astar_tests COMMAND scandium_astar_tests)

    add_executable(scandium_simulation_tests tests/simulation_tests.cpp)
    target_link_libraries(scandium_simulation_tests PRIVATE scandium_core)
    add_test(NAME scandium_simulation_tests COMMAND scandium_simulation_tests)

    add_executable(scandium_core_tests tests/core_tests.cpp)
    target_link_libraries(scandium_core_tests PRIVATE scandium_core)
    add_test(NAME scandium_core_tests COMMAND scandium_core_tests)

    add_executable(scandium_advanced_tests tests/advanced_tests.cpp)
    target_link_libraries(scandium_advanced_tests PRIVATE scandium_core)
    add_test(NAME scandium_advanced_tests COMMAND scandium_advanced_tests)

    add_executable(scandium_runtime_tests tests/runtime_tests.cpp)
    target_link_libraries(scandium_runtime_tests PRIVATE scandium_core)
    add_test(NAME scandium_runtime_tests COMMAND scandium_runtime_tests)

    add_executable(scandium_arena_tests tests/arena_tests.cpp)
    target_link_libraries(scandium_arena_tests PRIVATE scandium_core)
    add_test(NAME scandium_arena_tests COMMAND scandium_arena_tests)

    add_executable(scandium_network_tests tests/network_tests.cpp)
    target_link_libraries(scandium_network_tests PRIVATE scandium_core)
    add_test(NAME scandium_network_tests COMMAND scandium_network_tests)
endif()
