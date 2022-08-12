project(robot_model)

include_directories(include)

FIND_PACKAGE(OpenCV  REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(/usr/include/eigen3)

set(SOURCES 
    src/main.cpp
    src/robot.cpp)

set(HEADERS 
    include/robot.hpp)

add_executable(robot_model ${SOURCES} ${HEADERS})
target_link_libraries(robot_model ${OpenCV_LIBS})