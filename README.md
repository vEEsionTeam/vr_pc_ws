# 🕶️ VR System Setup Guide (Raspberry Pi + Host Computer)

## 🔌 Wired/Wireless Configuration

### 1️⃣ SSH into Raspberry Pi
```sh
ssh veesion@192.168.1.2
```

### 2️⃣ Update IP Address (if needed)
Check the IP address on the server:
```sh
hostname -I
```
Update the following line in both comm_ws/.../tf_client.py and gazebo_ws/.../tf_server.py:
```python
self.declare_parameter('server_ip', '192.168.x.x')
```
!!!*Build changes and source the workspace*!!!
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
../vr_scripts/tf_client.py --ros-args -p server_port:=5005 -p server_ip:=192.168. 
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
cd ~/gazebo_ws
source install/setup.bash
ros2 run tf_server tf_server --ros-args -p server_ip:=192.168.
```
#### 🖼️ Run RViz
```sh
cd ~/gazebo_ws
source install/setup.bash
ros2 launch rviz_sim rviz.launch.py
```
#### 🏞️ Launch Gazebo Environment
```sh
cd ~/gazebo_ws
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


![Screenshot from 2025-05-03 13-04-31](https://github.com/user-attachments/assets/d4ec60a3-9cbc-4058-ab6d-b5eacd94142d)



