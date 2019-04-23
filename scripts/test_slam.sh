#!/bin/sh
xterm -e " roslaunch basketbot world.launch"
sleep 5
#xterm -e " roslaunch basketbot mapping.launch "
#sleep 5
xterm -e " rosrun telep_twist_keyboard teleop_twist_keyboard.py"
