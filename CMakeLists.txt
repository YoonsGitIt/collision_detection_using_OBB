cmake_minimum_required(VERSION 3.23)
project(collisionDetectionOBB)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(collisionDetectionOBB src/CollisionTest.cpp src/SetObb.cpp src/IntersectionCalculator.cpp)
