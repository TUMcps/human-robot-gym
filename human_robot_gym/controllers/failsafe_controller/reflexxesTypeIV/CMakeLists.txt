cmake_minimum_required(VERSION 3.0.2)
project(reflexxes_type_iv)

add_compile_options(-std=c++17)

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
add_library(${PROJECT_NAME} ${PROJECT_SOURCES})

install(TARGETS ${PROJECT_NAME}
        DESTINATION /usr/lib)

## Mark cpp header files for installation
#install(DIRECTORY include/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})