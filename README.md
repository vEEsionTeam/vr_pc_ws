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
./vr_scripts/client.sh
```
#### 🧠 Run OpenVINS
```sh
./vr_scripts/camera.sh
```
### 💻 On Host Computer — 3 Terminals
#### 🛰️ Start Server Communication
```sh
cd ~/gazebo_ws
source install/setup.bash
ros2 run tf_server tf_server
```
#### 🖼️ Run RViz
```sh
cd ~/gazebo_ws
source install/setup.bash
ros2 launch vr_sim rviz.launch.py
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





