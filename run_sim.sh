#!/bin/bash

gnome-terminal --tab --title="SIMULATION" -- bash -c "source install/setup.bash;
export __NV_PRIME_RENDER_OFFLOAD=1;
export __GLX_VENDOR_LIBRARY_NAME=nvidia;
export __VK_LAYER_NV_optimus=NVIDIA_only;
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH;
ros2 launch arc_description gazebo.launch.py;
echo Press anykey to close;
read -n 1;"

sleep 10

gnome-terminal --tab --title="SET VIEW" -- bash -c '
gz service -s /gui/move_to/pose \
  --reqtype gz.msgs.GUICamera \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req "pose: {position:{x:1.7, y:2.0, z:2.0} orientation:{x:0.0, y:0.0, z:-0.6, w:0.27}}";'

sleep 3

gnome-terminal --tab --title="ACTUATOR" -- bash -c "source install/setup.bash;
ros2 run arc_navigation arc_nav;
echo Press anykey to close;
read -n 1;"