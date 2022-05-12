cmake_minimum_required(VERSION 3.0.2)
project(safety_shield)

## Compile as C++17
add_compile_options(-std=c++17)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

## Find catkin macros and libraries
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# Find Python
# IF YOU DON'T WANT TO USE CONDA, DISABLE THESE THREE LINES
#set(PYTHON_EXECUTABLE "$ENV{CONDA_PREFIX}/bin/python")
#set(PYTHON_INCLUDE_DIR "$ENV{CONDA_PREFIX}/include/python3.8")
#set(PYTHON_LIBRARY "$ENV{CONDA_PREFIX}/lib/libpython3.8.so")
#set(PYTHON_LIBRARY_DEBUG "$ENV{CONDA_PREFIX}/lib/libpython3.8.so")
#find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

include_directories(
  include
)

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/safety_shield.cc 
    src/long_term_traj.cc 
    src/motion.cc 
    src/path.cc 
    src/robot_reach.cc 
    src/human_reach.cc
    src/verify_iso.cc 
    #src/advanced_verify_iso.cc
    )

# Add external subdirectories
add_subdirectory(external/pybind11)
add_subdirectory(external/ReachLib)
add_subdirectory(external/reflexxesTypeIV)
add_subdirectory(external/spdlog)
add_subdirectory(external/yaml-cpp)

target_include_directories(${PROJECT_NAME}
    PUBLIC external/pybind11/include
    PUBLIC external/ReachLib/include
    PUBLIC external/reflexxesTypeIV/include
    PUBLIC external/spdlog/include
    PUBLIC external/yaml-cpp/include
)

target_link_directories(${PROJECT_NAME}
    PUBLIC external/ReachLib/src
    PUBLIC external/reflexxesTypeIV/src
    PUBLIC external/spdlog/src
    PUBLIC external/yaml-cpp/src
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen SaRA reflexxes_type_iv spdlog yaml-cpp)

install(TARGETS ${PROJECT_NAME} DESTINATION /usr/lib)

add_executable(debug src/debug.cc)
target_link_libraries(debug ${PROJECT_NAME} reflexxes_type_iv spdlog Eigen3::Eigen yaml-cpp)
#############
## GTest ##
#############
enable_testing()
find_package(GTest REQUIRED)
include(GoogleTest)

add_executable(tests tests/src/long_term_traj_tests.cc)
target_include_directories(tests PUBLIC tests/include)
target_link_libraries(tests ${PROJECT_NAME} GTest::GTest GTest::Main)
gtest_discover_tests(tests)

#catkin_add_gtest(verify_iso_test tests/capsule_tests.cc tests/verify_iso_fixture.h)
#target_link_libraries(verify_iso_test ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

#catkin_add_gtest(long_term_traj_tests tests/long_term_traj_tests.cc tests/long_term_traj_fixture.h)
#target_link_libraries(long_term_traj_tests ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

##############
## Bindings ##
##############
pybind11_add_module(safety_shield_py MODULE bindings/safety_shield_bindings.cpp)
target_link_libraries(safety_shield_py PRIVATE ${PROJECT_NAME} reflexxes_type_iv spdlog Eigen3::Eigen yaml-cpp)
#add_executable(safety_shield_bindings bindings/safety_shield_bindings.cc)