cmake_minimum_required(VERSION 3.0.2)
project(safety_shield)

## Compile as C++17
add_compile_options(-std=c++17)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

## Find catkin macros and libraries
find_package(spdlog REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
find_package(SaRA REQUIRED)
find_package(reflexxes_type_iv REQUIRED)

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

target_link_libraries(${PROJECT_NAME} spdlog Eigen3::Eigen yaml-cpp)
target_link_libraries(${PROJECT_NAME} SaRA)
target_link_libraries(${PROJECT_NAME} reflexxes_type_iv)

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
find_package(Python COMPONENTS Interpreter Development REQUIRED)
pybind11_add_module(safety_shield_py MODULE bindings/safety_shield_bindings.cpp)
target_link_libraries(safety_shield_py PRIVATE ${PROJECT_NAME} reflexxes_type_iv spdlog Eigen3::Eigen yaml-cpp)
#add_executable(safety_shield_bindings bindings/safety_shield_bindings.cc)