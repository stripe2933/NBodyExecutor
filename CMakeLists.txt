cmake_minimum_required(VERSION 3.11)
project(NBodyExecutor LANGUAGES CXX)

if (NOT DEFINED NBODY_EXECUTOR_BUILD_EXAMPLE)
    set(NBODY_EXECUTOR_BUILD_EXAMPLE OFF)
endif()

if (NOT DEFINED NBODY_EXECUTOR_USE_FAST_MATH)
    set(NBODY_EXECUTOR_USE_FAST_MATH ON)
endif()

find_package(glm CONFIG REQUIRED)
find_path(BSHOSHANY_THREAD_POOL_INCLUDE_DIRS "BS_thread_pool.hpp")

add_library(NBodyExecutor STATIC
    lib/NaiveExecutor.cpp
    lib/BarnesHutExecutor.cpp
    lib/OctTree.cpp
    lib/Cube.cpp
)
target_include_directories(NBodyExecutor
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${BSHOSHANY_THREAD_POOL_INCLUDE_DIRS}
)
target_link_directories(NBodyExecutor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib)
target_compile_features(NBodyExecutor PUBLIC cxx_std_20)
if (NBODY_EXECUTOR_USE_FAST_MATH)
    target_compile_options(NBodyExecutor PUBLIC -ffast-math)
endif()

# Build examples if requested.
if (NBODY_EXECUTOR_BUILD_EXAMPLE)
    add_subdirectory(example)
endif()