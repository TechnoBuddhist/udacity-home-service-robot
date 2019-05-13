cmake_minimum_required(VERSION 2.8.3)
project(marker_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  action.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs visualization_msgs message_runtime
)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}

)

add_executable(${PROJECT_NAME} src/marker_manager.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
