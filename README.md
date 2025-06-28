# VR Tracking System

This repository includes the **PC-side** components of the VR tracking system, providing a GUI setup and ROS 2 Humble & Gazebo based simulation with a custom head-mounted model for Gazebo and a Python GUI to launch tracking components via SSH.

To run the project successfully, you also need to run the companion **[vr_rpi_ws](https://github.com/vEEsionTeam/vr_rpi_ws)** repository on a Raspberry Pi equipped with a camera and IMU.

You can watch the project video here: [https://youtu.be/1OSWoaaN1xk?si=DFQOnIu5OELyaWig](https://youtu.be/1OSWoaaN1xk?si=DFQOnIu5OELyaWig)

---

## ✅ Requirements

- ROS 2 Humble installed and sourced
- Gazebo
- Python 3 with PyQt5 for the GUI

---

## 🔧 Setup Instructions

### 1. Clone this repository

```sh
git clone https://github.com/vEEsionTeam/vr_pc_ws.git
cd vr_pc_ws
```

### 2. Update the URDF mesh path
Edit the mesh path in the URDF file:
```sh
cd src/gazebo_sim/urdf
nano head.urdf
```
Replace the mesh path line with your absolute path:
```xml
<mesh filename="/absolute/path/to/vr_pc_ws/src/gazebo_sim/meshes/head.stl" scale="0.0001 0.0001 0.0001"/>
```
### 3. Build the workspace

```sh
cd ~/vr_pc_ws
colcon build
source install/setup.bash
```
### 4. Set GAZEBO_MODEL_PATH
To use the custom models in Gazebo, add the following line to the end of your ~/.bashrc file:
```sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/absolute/path/to/vr_pc_ws/src/gazebo_sim/models
```

Then apply the changes:
```sh
source ~/.bashrc
```

# ▶️ Run the GUI
Navigate to the GUI directory 
```sh
cd ~/vr_pc_ws/vr_gui
```
and run 
```sh
python3 main.py
```


### 📝 Notes
- Make sure all ROS2 packages are built after any change.
- Always source the workspace before running any ROS 2 commands.
