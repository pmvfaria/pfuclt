cmake_minimum_required(VERSION 3.8)
project(pfuclt)

# Warn to add optimizations, remove when done
message(AUTHOR_WARNING "Remember to change to Release")

option(PFUCLT_SIMD_OPTIMIZATIONS "Enable CPU-specific SIMD optimizations (SSE/AVX or NEON, e.g.)" ON)
option(PFUCLT_VECTORIZE_DEBUG "Enable options to help in spotting missed vectorization in loops" OFF)
option(ENABLE_AES "Enable AES optimizations (enabled by default with PFUCLT_SIMD_OPTIMIZATIONS, set OFF to disable" ON)
option(ENABLE_AVX2 "Enable AVX2 optimizations" ON)


# Debug until ready for release
set(CMAKE_BUILD_TYPE Release)
set(PFUCLT_VECTORIZE_DEBUG OFF)
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

# C++ 17, GNU 8.1
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.1)
        message(FATAL_ERROR "GNU g++ minimum version 8.1 is required")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -Wpedantic")
        set(CMAKE_CXX_STANDARD 17)
        set(CMAKE_CXX_STANDARD_REQUIRED ON)
        add_definitions(-DCOMPILEDWITHC17)
    endif()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    message(WARN "Untested with CLang, please use GNU g++")
else()
    message(WARN "Unknown compiler, please use GNU g++")
endif ()

if(PFUCLT_VECTORIZE_DEBUG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopt-info-vec-optimized -fopt-info-vec-missed")
endif()


# OpenMP needed for parallel STL
# One could add the -D_GLIBCXX_PARALLEL flag, but that turns all sequential STL algorithms into parallel (if available)
option(USE_OpenMP "Use OpenMP" ON)
if(USE_OpenMP)
    find_package(OpenMP)
    if(OPENMP_FOUND)
        message(STATUS "OPENMP FOUND")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(OpenMP_LIBS gomp)
    else()
        message(WARN "OpenMP not found, it is advised to install it")
    endif()
endif()

# SIMD optimization
if (CMAKE_SYSTEM_PROCESSOR MATCHES "^(i.86|x86(_64)?)$")
    if (PFUCLT_SIMD_OPTIMIZATIONS)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4")
        if (ENABLE_AES)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -maes")
        endif ()
    elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2")
    endif()

    if (ENABLE_AVX2)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2 -mbmi -mbmi2 -maes")
        if (PERMUTE_WITH_GATHER)
            add_definitions(-DPERMUTE_WITH_GATHER)
        elseif (PERMUTE_WITH_SHUFFLES)
            add_definitions(-DPERMUTE_WITH_SHUFFLES)
        endif()
    endif()
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    clt_msgs
    tf2
    tf2_ros
)

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS roscpp
    DEPENDS
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    src
)

# StopWatch library
add_library(stopwatch src/thirdparty/stopwatch/StopWatch.cpp)

add_executable(pfuclt_node src/ros/pfuclt_node.cpp
        src/ros/pfuclt_publisher.cpp
        src/ros/debug.cpp
        src/particle/subparticle.cpp
        src/particle/particles.cpp
        src/map/landmark_map.cpp
        src/map/map_ros.cpp
        src/pfuclt/pfuclt.cpp
        src/pfuclt/initialize.cpp
        src/pfuclt/state.cpp
        src/pfuclt/robot.cpp
        src/pfuclt/target.cpp
        src/sensor/odometry_data.cpp
        src/sensor/measurements_data.cpp
        src/thirdparty/stopwatch/StopWatch.cpp
)
target_link_libraries(pfuclt_node ${catkin_LIBRARIES})

add_executable(test_pararallelization test/test_parallelization.cpp src/particle/particles.cpp src/particle/subparticle.cpp)
target_link_libraries(test_pararallelization ${catkin_LIBRARIES} stopwatch)

# Google Tests
catkin_add_gtest(test_particles test/test_particle.cpp src/particle/particles.cpp src/particle/subparticle.cpp)
target_link_libraries(test_particles ${catkin_LIBRARIES})

catkin_add_gtest(test_state test/test_state.cpp src/pfuclt/state.cpp src/particle/particles.cpp src/particle/subparticle.cpp)
target_link_libraries(test_state ${catkin_LIBRARIES})

catkin_add_gtest(test_landmark_map test/test_landmark_map.cpp src/map/landmark_map.cpp)
target_link_libraries(test_particles ${catkin_LIBRARIES})

catkin_add_gtest(test_miscellaneous test/test_miscellaneous.cpp src/particle/subparticle.cpp src/pfuclt/state.hpp)
target_link_libraries(test_miscellaneous ${catkin_LIBRARIES} stopwatch)


MESSAGE(STATUS "Compiler cxx debug flags:" ${CMAKE_CXX_FLAGS_DEBUG})
MESSAGE(STATUS "Compiler cxx release flags:" ${CMAKE_CXX_FLAGS_RELEASE})
MESSAGE(STATUS "Compiler cxx min size flags:" ${CMAKE_CXX_FLAGS_MINSIZEREL})
MESSAGE(STATUS "Compiler cxx flags:" ${CMAKE_CXX_FLAGS})
