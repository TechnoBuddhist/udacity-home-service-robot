cmake_minimum_required(VERSION 2.8.3)
project(home_service)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  marker_manager
  move_base_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib geometry_msgs marker_manager move_base_msgs roscpp std_msgs message_runtime marker_manager
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/home_service.cpp)
add_dependencies(marker_manager marker_manager_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})