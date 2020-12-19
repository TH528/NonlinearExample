cmake_minimum_required(VERSION 3.1)
project(ControlSystemSolver CXX)

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-O2")
set(CMAKE_CXX_FLAGS "-std=c++17")

include_directories(solver)
include_directories(csv)

add_executable(main main.cpp)