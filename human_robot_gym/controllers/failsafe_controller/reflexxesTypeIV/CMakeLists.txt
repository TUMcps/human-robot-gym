cmake_minimum_required(VERSION 3.0.2)
project(reflexxes_type_iv)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-std=c++17)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

include(GNUInstallDirs)

include_directories(
  include
)

set(PROJECT_SOURCES
    src/ReflexxesAPI/ReflexxesAPI.cpp
    src/ReflexxesAPI/TypeIVRML_C_Wrapper.cpp
    src/TypeIVRMLMath/TypeIVRMLABK.cpp                  
    src/TypeIVRMLMath/TypeIVRMLStep1Decisions.cpp
    src/TypeIVRMLMath/TypeIVRMLDecisionTree1A.cpp       
    src/TypeIVRMLMath/TypeIVRMLStep1IntermediateChangingProfiles.cpp
    src/TypeIVRMLMath/TypeIVRMLDecisionTree1B1.cpp      
    src/TypeIVRMLMath/TypeIVRMLStep1IntermediateTimeProfiles.cpp
    src/TypeIVRMLMath/TypeIVRMLDecisionTree1B2.cpp      
    src/TypeIVRMLMath/TypeIVRMLStep1Profiles.cpp
    src/TypeIVRMLMath/TypeIVRMLDecisionTree1B3.cpp      
    src/TypeIVRMLMath/TypeIVRMLStep1RootFunctions.cpp
    src/TypeIVRMLMath/TypeIVRMLDecisionTree1C.cpp       
    src/TypeIVRMLMath/TypeIVRMLStep2Decisions.cpp
    src/TypeIVRMLMath/TypeIVRMLDecisionTree2.cpp        
    src/TypeIVRMLMath/TypeIVRMLStep2IntermediateProfiles.cpp
    src/TypeIVRMLMath/TypeIVRMLMath.cpp                 
    src/TypeIVRMLMath/TypeIVRMLStep2Profiles.cpp
    src/TypeIVRMLMath/TypeIVRMLMovingAverageFilter.cpp  
    src/TypeIVRMLMath/TypeIVRMLStep2RootFunctions.cpp
    src/TypeIVRMLMath/TypeIVRMLPolynomial.cpp           
    src/TypeIVRMLMath/TypeIVRMLStep2WithoutSynchronization.cpp
    src/TypeIVRMLMath/TypeIVRMLQuicksort.cpp
    src/TypeIVRMLPosition/TypeIVRMLCalculatePositionalExtrems.cpp      
    src/TypeIVRMLPosition/TypeIVRMLStep1.cpp
    src/TypeIVRMLPosition/TypeIVRMLCalculateScalingVector.cpp          
    src/TypeIVRMLPosition/TypeIVRMLStep2.cpp
    src/TypeIVRMLPosition/TypeIVRMLFallBackStrategy.cpp                
    src/TypeIVRMLPosition/TypeIVRMLStep2PhaseSynchronization.cpp
    src/TypeIVRMLPosition/TypeIVRMLIsPhaseSynchronizationPossible.cpp  
    src/TypeIVRMLPosition/TypeIVRMLStep3.cpp
    src/TypeIVRMLPosition/TypeIVRMLPosition.cpp                        
    src/TypeIVRMLPosition/TypeIVRMLThreadFunction.cpp
    src/TypeIVRMLPosition/TypeIVRMLSetupModifiedSelectionVector.cpp
    src/TypeIVRMLVelocity/TypeIVRMLVelocityCalculatePositionalExtrems.cpp  
    src/TypeIVRMLVelocity/TypeIVRMLVelocityIsPhaseSynchronizationPossible.cpp
    src/TypeIVRMLVelocity/TypeIVRMLVelocity.cpp                            
    src/TypeIVRMLVelocity/TypeIVRMLVelocityMethods.cpp
    src/TypeIVRMLVelocity/TypeIVRMLVelocityFallBackStrategy.cpp            
    src/TypeIVRMLVelocity/TypeIVRMLVelocitySetupPhaseSyncSelectionVector.cpp)
add_library(${PROJECT_NAME} SHARED ${PROJECT_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )#PRIVATE src)

set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION 1.0.0
    SOVERSION 1)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Config
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

install(EXPORT ${PROJECT_NAME}Config DESTINATION share/${PROJECT_NAME}/cmake)

export(TARGETS ${PROJECT_NAME} FILE ${PROJECT_NAME}Config.cmake)

## Mark cpp header files for installation
#install(DIRECTORY include/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})