cmake_minimum_required(VERSION 2.8.3)
project(people_tracking_filter)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  people_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  liborocos-bfl
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        geometry_msgs
        message_filters
        people_msgs
        roscpp
        sensor_msgs
        std_msgs
        tf
        liborocos-bfl
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
            src/uniform_vector.cpp
            src/gaussian_vector.cpp
            src/gaussian_pos_vel.cpp
            src/mcpdf_pos_vel.cpp
            src/mcpdf_vector.cpp
            src/sysmodel_pos_vel.cpp
            src/sysmodel_vector.cpp
            src/measmodel_pos.cpp
            src/measmodel_vector.cpp
            src/tracker_particle.cpp
            src/tracker_kalman.cpp
            src/detector_particle.cpp
)
add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES})

add_executable(people_tracker src/people_tracking_node.cpp)
add_dependencies(people_tracker people_tracking_filter ${catkin_EXPORTED_TARGETS})
target_link_libraries(people_tracker
   people_tracking_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES}
)



install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS people_tracker
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
