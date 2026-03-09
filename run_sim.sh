#!/bin/bash

gnome-terminal --tab --title="SIMULATION" -- bash -c "source install/setup.bash;
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH;
ros2 launch lily_description gazebo.launch.py; 
echo Press anykey to close;
read -n 1;"