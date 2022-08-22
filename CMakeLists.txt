cmake_minimum_required(VERSION 3.16)

project(robot_model)

set(CMAKE_BUILD_TYPE Debug) 

include_directories(include)

FIND_PACKAGE(OpenCV  REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(/usr/include/eigen3)

set(SOURCES 
    src/main.cpp
    src/robot.cpp
    src/rangefinder.cpp)

set(HEADERS 
    include/robot.hpp
    include/rangefinder.hpp)

add_executable(robot_model ${SOURCES} ${HEADERS})
target_link_libraries(robot_model ${OpenCV_LIBS})

# add_library(robot_model SHARED ${SOURCES} ${HEADERS})
# target_link_libraries(robot_model ${OpenCV_LIBS})