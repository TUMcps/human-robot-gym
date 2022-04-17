cmake_minimum_required(VERSION 3.0.2)

project(failsafe_controller)

include(CTest)

add_subdirectory(yaml-cpp)
add_subdirectory(ReachLib)
add_subdirectory(reflexxesTypeIV)
add_subdirectory(pybind11)
add_subdirectory(safety_shield)