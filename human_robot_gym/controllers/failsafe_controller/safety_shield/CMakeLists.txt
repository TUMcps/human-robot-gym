cmake_minimum_required(VERSION 3.0.2)
project(safety_shield)

## Compile as C++17
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(spdlog REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(SaRA REQUIRED)
find_package(reflexxes_type_iv REQUIRED)

include_directories(
  include
)

## Declare a C++ library
add_library(${PROJECT_NAME}
    #src/safety_shield.cc 
    src/long_term_traj.cc 
    src/motion.cc 
    src/path.cc 
    src/robot_reach.cc 
    #src/human_reach.cc
    src/verify_iso.cc 
    #src/advanced_verify_iso.cc
    )

target_link_libraries(${PROJECT_NAME} spdlog Eigen3::Eigen)

target_link_libraries(${PROJECT_NAME} SaRA)
target_link_libraries(${PROJECT_NAME} reflexxes_type_iv)

install(TARGETS ${PROJECT_NAME} DESTINATION /usr/lib)
#############
## GTest ##
#############
#catkin_add_gtest(verify_iso_test tests/capsule_tests.cc tests/verify_iso_fixture.h)
#target_link_libraries(verify_iso_test ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

#catkin_add_gtest(long_term_traj_tests tests/long_term_traj_tests.cc tests/long_term_traj_fixture.h)
#target_link_libraries(long_term_traj_tests ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)