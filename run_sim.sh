#!/bin/bash

gnome-terminal --tab --title="SIMULATION" -- bash -c "source install/setup.bash;
ros2 launch lily_description gazebo.launch.py; 
echo Press anykey to close;
read -n 1;"