gnome-terminal --tab --title="Learning" -- bash -c "export PYTHONPATH="/home/marcel/arc/lib/python3.12/site-packages:$PYTHONPATH";
source install/setup.bash;
ros2 run arc_learning train;
echo Press anykey to close;
read -n 1;"
