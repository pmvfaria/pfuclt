cmake_minimum_required(VERSION 3.8)
project(pfuclt)

# Warn to add optimizations, remove when done
message(AUTHOR_WARNING "Rember to change to Release and add -O3 optimizations to compiler")

# Debug until ready for release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif ()
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

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

# C++ 17, GNU 8.1
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.1)
        message(FATAL_ERROR "GNU g++ minimum version 8.1 is required")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")
        set(CMAKE_CXX_STANDARD 17)
        set(CMAKE_CXX_STANDARD_REQUIRED ON)
        add_definitions(-DCOMPILEDWITHC17)
    endif()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    message(WARN "Untested with CLang, please use GNU g++")
else()
    message(WARN "Unknown compiler, please use GNU g++")
endif ()

# All warnings, add optimizations later
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

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

add_executable(pfuclt_node src/ros/pfuclt_node.cpp src/sensor/odometry_data.cpp src/particle/particles.cpp src/particle/particles.hpp
        src/map/landmark_map.cpp src/map/landmark_map.hpp src/map/map_ros.cpp src/map/map_ros.hpp
        src/sensor/landmark_data.cpp src/pfuclt/pfuclt.cpp src/pfuclt/pfuclt.hpp
        src/pfuclt/initialize.cpp src/pfuclt/state.cpp src/pfuclt/state.hpp src/pfuclt/robot.cpp src/pfuclt/robot.hpp
        src/thirdparty/stopwatch/StopWatch.cpp
        src/thirdparty/CTPL/ctpl_stl.h)
target_link_libraries(pfuclt_node ${catkin_LIBRARIES})

add_executable(test_pararallelization test/test_parallelization.cpp src/particle/particles.cpp)
target_link_libraries(test_pararallelization ${catkin_LIBRARIES} stopwatch)

# Google Tests
catkin_add_gtest(test_particles test/test_particle.cpp src/particle/particles.cpp src/particle/subparticle.hpp)
target_link_libraries(test_particles ${catkin_LIBRARIES})

catkin_add_gtest(test_landmark_map test/test_landmark_map.cpp src/map/landmark_map.cpp src/map/landmark_map.hpp)
target_link_libraries(test_particles ${catkin_LIBRARIES})
