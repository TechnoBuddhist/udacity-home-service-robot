cmake_minimum_required(VERSION 2.8.3)
project(pick_objects)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
)

include_directories(
# include
  src
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/pick_objects_node.cpp)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)