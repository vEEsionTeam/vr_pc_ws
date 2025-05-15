# 🕶️ VR System Setup Guide (Raspberry Pi + Host Computer)

## 🔌 Wired/Wireless Configuration

### SSH into Raspberry Pi
Check the RPI IP
```sh
ssh veesion@192.168.1.2
```
or 
```sh
ssh veesion@192.168.1.84
```
## Terminal Launch Instructions (7 Terminals Total)
### 🍓 On Raspberry Pi — 4 Terminals
#### 📷 Run Camera Node
```sh
./vr_scripts/camera.sh
```
#### 📡 Run IMU Node
```sh
./vr_scripts/imu.sh
```
#### 🌐 Run Client Communication
```sh
./vr_scripts/tf_client2.py --ros-args -p img_enable:=1 -p path_enable:=0 -p points_enable:=0 server_ip:=192.168.  
```
!!! CHANGE SERVER_IP
Check the IP address on the server:
```sh
hostname -I
```
#### 🧠 Run OpenVINS
```sh
./vr_scripts/openvins.sh
```

#### Patch Recording
```sh
ros2 run ov_eval pose_to_file_ros2 --ros-args -p topic:=/ov_msckf/odomimu -p topic_type:=Odometry -p output:=/home/veesion/veesion_ws/path_recording/test1.txt
```
### 💻 On Host Computer — 3 Terminals
#### 🛰️ Start Server Communication
```sh
cd ~/vr_pc_ws
source install/setup.bash
ros2 run tf_server tf_server2 --ros-args -p img_enable:=1 -p path_enable:=0 -p points_enable:=0  -p server_ip:=192.168. 
```
!!! CHANGE SERVER_IP
#### 🖼️ Run RViz
```sh
cd ~/vr_pc_ws
source install/setup.bash
ros2 launch rviz_sim rviz.launch.py
```
#### 🏞️ Launch Gazebo Environment
```sh
cd ~/vr_pc_ws
source install/setup.bash
ros2 launch vr_sim gazebo.launch.py
```
#### 📝 Notes
Make sure all ROS 2 packages are built using:
```sh
colcon build
```
Always source the workspace before running any ROS 2 commands:
```sh
source install/setup.bash
```

Consider using terminal multiplexers like terminator for efficient multi-terminal management.
##### Copy a file form RPI to PC
```sh
scp veesion@192.168.1.2:/home/veesion/veesion_ws/path_recording/testx1.txt ~/Desktop/paths/
```


