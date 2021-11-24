cmake_minimum_required(VERSION 3.0.2)
project(scuedk_bridge)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        tf
        scuedk
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(scuedk_bridge
        src/main.cpp
        src/Node.cpp
        )
target_link_libraries(scuedk_bridge ${catkin_LIBRARIES})