cmake_minimum_required(VERSION 3.16)
project(robot_env_swig LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})

find_package(Python3 REQUIRED COMPONENTS Development)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${Python3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(CMAKE_SWIG_FLAGS "")

set_source_files_properties(swig/robot_env.i PROPERTIES CPLUSPLUS ON)
set_source_files_properties(swig/robot_env.i PROPERTIES SWIG_FLAGS "-py3")

swig_add_library(robot_env_cpp
    TYPE MODULE
    LANGUAGE python
    SOURCES swig/robot_env.i
            ../src/RobotEnv.cpp
)

swig_link_libraries(robot_env_cpp
    ${Python3_LIBRARIES}
    ${OpenCV_LIBS}
    Eigen3::Eigen
    mobile_robot_model
)


