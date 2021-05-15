#!/bin/bash

gnome-terminal --tab --title="ros1_gazebo" -- bash -c "source ros.sh; roslaunch rt2_assignment1 vrepSim.launch"
gnome-terminal --tab --title="bridge" -- bash -c "sleep 3; source ros12.sh; ros2 run ros1_bridge dynamic_bridge"
gnome-terminal --tab --title="ros2_component" -- bash -c "sleep 4; source ros2.sh; ros2 launch rt2_assignment1 my_launch.py"
