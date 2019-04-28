#!/bin/sh
echo "    roslaunch turtlebot_gazebo turtlebot_world.launch ..."
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
echo "    roslaunch turtlebot_gazebo amcl_demo.launch ..."
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
echo "    roslaunch turtlebot_rviz_launchers view_navigation.launch ..."
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch"
