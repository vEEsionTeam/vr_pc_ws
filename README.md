# VR Tracking System

This repository provides a GUI setup and ROS 2 Humble&Gazebo based simulation for a VR tracking system. It includes a custom head-mounted model for Gazebo and a Python GUI for launching tracking components on RPI with SSH.

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

Make sure all ROS2 packages are built after any change:
```sh
colcon build
```
Always source the workspace before running any ROS 2 commands:
```sh
source install/setup.bash
```
