cmake_minimum_required(VERSION 3.16)
project(motion_planner
    VERSION 1.0.0
    DESCRIPTION "2D Sampling-Based Motion Planning Library (RRT / RRT*)"
    LANGUAGES CXX
)

# ── C++17 standard ─────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Compiler warnings (help catch bugs early) ─────────────────
add_compile_options(-Wall -Wextra -Wpedantic)

# ── Library ────────────────────────────────────────────────────
# Build the core motion planning library as a static library.
add_library(motion_planner_lib STATIC
    src/environment.cpp
    src/rrt.cpp
    src/rrt_star.cpp
    src/path_smoother.cpp
    src/benchmark.cpp
)
target_include_directories(motion_planner_lib PUBLIC include)

# ── Main executable ───────────────────────────────────────────
add_executable(motion_planner src/main.cpp)
target_link_libraries(motion_planner PRIVATE motion_planner_lib)

# ── Tests (GoogleTest) ────────────────────────────────────────
# Only build tests if GoogleTest is found.
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    # Fallback: try to find gtest via pkg-config or system headers.
    find_library(GTEST_LIB gtest)
    find_library(GTEST_MAIN_LIB gtest_main)
    if(GTEST_LIB AND GTEST_MAIN_LIB)
        set(GTEST_FOUND TRUE)
    endif()
endif()

if(GTest_FOUND OR GTEST_LIB)
    enable_testing()

    # Helper function to add a test executable.
    function(add_mp_test test_name test_source)
        add_executable(${test_name} ${test_source})
        target_link_libraries(${test_name} PRIVATE
            motion_planner_lib
            GTest::gtest GTest::gtest_main
            pthread
        )
        add_test(NAME ${test_name} COMMAND ${test_name})
    endfunction()

    add_mp_test(test_types        tests/test_types.cpp)
    add_mp_test(test_environment  tests/test_environment.cpp)
    add_mp_test(test_rrt          tests/test_rrt.cpp)
    add_mp_test(test_rrt_star     tests/test_rrt_star.cpp)
    add_mp_test(test_path_smoother tests/test_path_smoother.cpp)

    message(STATUS "Tests enabled (GoogleTest found)")
else()
    message(WARNING "GoogleTest not found — tests will not be built")
endif()
