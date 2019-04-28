#!/bin/sh
echo "    roslaunch basketbot world.launch ..."
xterm -e " roslaunch basketbot world.launch" &
sleep 5
echo "    roslaunch basketbot mapping.launch ..."
xterm -e " roslaunch basketbot mapping.launch" &
sleep 5
echo "    rosrun teleop_twist_keyboard teleop_twist_keyboard.py ..."
xterm -e " rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
