#!/bin/bash

gnome-terminal --tab --title="MODEL" -- bash -c "source install/setup.bash;
ros2 launch lily_description description.launch.py rviz:=true; 
echo Press anykey to close;
read -n 1;"