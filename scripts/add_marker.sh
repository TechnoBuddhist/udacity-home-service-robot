#!/bin/sh
echo "    roslaunch turtlebot_gazebo turtlebot_world.launch ..."
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/ros/ros/home-service-robot/src/basketbot/world/RoboWorld2.world
" &
sleep 5
echo "    roslaunch turtlebot_gazebo amcl_demo.launch ..."
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/ros/ros/home-service-robot/src/basketbot/maps/map.yaml" &
sleep 5
echo "    roslaunch turtlebot_rviz_launchers view_navigation.launch ..."
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
echo "    launching - add_marker node ..."
xterm -e " rosrun add_markers add_markers_node"