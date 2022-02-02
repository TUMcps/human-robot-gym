cmake_minimum_required(VERSION 2.8.9)
project(SaRA)
set(CMAKE_BUILD_TYPE Release)

#Bring the headers, such as Student.h into the project
include_directories(include)

#However, the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "src/*.cpp")
#set(SOURCES src/Point.cpp src/Capsule.cpp)

#Generate the shared library from the sources [SHARED]
add_library(SaRA STATIC ${SOURCES})

#Set the location for library installation -- i.e., /usr/lib in this case
# not really necessary in this example. Use "sudo make install" to apply
install(TARGETS SaRA DESTINATION /usr/lib)