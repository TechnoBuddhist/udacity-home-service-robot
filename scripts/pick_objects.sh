#!/bin/sh
echo "    launching - basketbot world.launch ..."
xterm -e " roslaunch basketbot world.launch" &
sleep 5
echo "    launching - basketbot mapping.launch ..."
xterm -e " roslaunch basketbot mapping.launch" &
sleep 5
echo "    launching - basketbot view_navigation.launch ..."
xterm -e " roslaunch basketbot view_navigation.launch" &
sleep 5
echo "    launching - pick_objects pick_objects_node ..."
xterm -e " rosrun pick_objects pick_objects_node"
