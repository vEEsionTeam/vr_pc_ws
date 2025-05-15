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
### 💻 On Host Computer — 3 Terminals

#### 🍓📷 Run Camera Node
```sh
./vr_scripts/camera.sh
```
#### 🍓📡 Run IMU Node
```sh
./vr_scripts/imu.sh
```
#### 💻🛰️ Start Server Communication
```sh
./server.sh --img 0 --path 0 --points 0 --ip 192.168. 
```
!!! CHANGE SERVER_IP

#### 🍓🌐 Run Client Communication
```sh
./vr_scripts/tf_client2.py --ros-args -p img_enable:=1 -p path_enable:=0 -p points_enable:=0 -p server_ip:=192.168.  
```
!!! CHANGE SERVER_IP
Check the IP address on the server:
```sh
hostname -I
```

#### 💻🖼️ Run RViz
```sh
./rviz.sh
```
#### 💻🏞️ Launch Gazebo Environment
```sh
./gazebo.sh
```

#### 🍓🧠 Run OpenVINS
```sh
./vr_scripts/openvins.sh
```

### 📝 Notes

#### 🍓 Path Recording
```sh
ros2 run ov_eval pose_to_file_ros2 --ros-args -p topic:=/ov_msckf/odomimu -p topic_type:=Odometry -p output:=/home/veesion/veesion_ws/path_recording/test1.txt
```

Make sure all ROS 2 packages are built after any change:
```sh
colcon build
```
Always source the workspace before running any ROS 2 commands:
```sh
source install/setup.bash
```

Consider using terminal multiplexers like terminator for efficient multi-terminal management.
#### 💻 Copy a file form RPI to PC
```sh
scp veesion@192.168.1.2:/home/veesion/veesion_ws/path_recording/testx1.txt ~/Desktop/paths/
```
#### Additional Commands
```sh
ros2 launch ov_msckf veesion1.launch.py config:=veesion3 rviz_enable:=false
```


