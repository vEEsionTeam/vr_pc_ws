cd ~/vr_pc_ws || exit
source install/setup.bash
ros2 launch gazebo_sim gazebo.launch.py world:=Cafe.world
