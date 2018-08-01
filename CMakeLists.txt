cmake_minimum_required(VERSION 3.8)
project(pfuclt)

# Warn to add optimizations, remove when done
message(WARNING "Rember to change to Release and add -O3 optimizations to compiler")

# Debug until ready for release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif ()
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

# OpenMP needed for parallel STL
# One could add the -D_GLIBCXX_PARALLEL flag, but that turns all sequential STL algorithms into parallel (if available)
OPTION(USE_OpenMP "Use OpenMP" ON)
IF(USE_OpenMP)
    FIND_PACKAGE(OpenMP)
    IF(OPENMP_FOUND)
        message(STATUS "OPENMP FOUND")
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        SET(OpenMP_LIBS gomp)
    ENDIF()
ENDIF()

# C++ 17
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if (COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Compiler supports C++17, using -std=c++17 flag.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif ()

# All warnings, add optimizations later
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

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

add_executable(pfuclt src/pfuclt/pfuclt.cpp src/data/odometry.cpp src/particle/particle.cpp)
target_link_libraries(pfuclt ${catkin_LIBRARIES})

add_executable(test_pararallelization test/test_parallelization.cpp src/particle/particle.cpp)
target_link_libraries(test_pararallelization ${catkin_LIBRARIES} stopwatch)

catkin_add_gtest(test_particles test/test_particle.cpp src/particle/particle.cpp src/particle/subparticle.hpp)
target_link_libraries(test_particles ${catkin_LIBRARIES})
