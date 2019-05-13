cmake_minimum_required(VERSION 2.8.3)
project(add_markers)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  marker_manager
)

catkin_package(
  CATKIN_DEPENDS roscpp visualization_msgs message_runtime marker_manager
)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
)

add_dependencies(marker_manager marker_manager_generate_messages_cpp)
add_executable(${PROJECT_NAME} src/add_markers.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})