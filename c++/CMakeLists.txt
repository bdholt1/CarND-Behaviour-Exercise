cmake_minimum_required (VERSION 3.5)

set (CMAKE_CXX_STANDARD 11)
project (BehaviourPlanner)
add_executable(planner main.cpp vehicle.cpp cost_function.cpp road.cpp)