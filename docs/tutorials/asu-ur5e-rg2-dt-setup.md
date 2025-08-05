# UR5e + RG2 Gripper Setup (Sim + Real World)

This tutorial walks you through setting up the UR5e robot with an RG2 gripper for both Isaac Sim (4.2 and 4.5) and a physical robot, including control, ROS 2 integration, and URSim.

---

## Part 1: Setup Extension in Isaac Sim

### Clone the Extension

```bash
git clone https://github.com/inbarajaldrin/vlm_isaac_sim
```

### Add Extension Path in Isaac Sim Settings

For Isaac Sim **4.2**:
```
/home/{your_username}/Documents/vlm_isaac_sim/isaac_exts/asu-ur5e-dt/exts
```

For Isaac Sim **4.5**:
```
/home/{your_username}/Documents/vlm_isaac_sim/isaac_exts/
```

> Go crazy on the UI buttons.

---

## Part 2: URSim (Simulation of Real Robot)

### Run URSim Container

```bash
docker run --rm -it \
  -p 5900:5900 -p 6080:6080 \
  -v ${HOME}/.ursim/urcaps:/urcaps \
  -v ${HOME}/.ursim/programs:/ursim/programs \
  --name ursim universalrobots/ursim_e-series
```

---

## Part 3: Bringup in ROS 2

### UR5e in Simulation (URSim)

```bash
ros2 launch ur_bringup ur5e.launch.py ur_type:=ur5e robot_ip:=172.17.0.2
```

### UR5e in Real Hardware

```bash
ros2 launch ur_bringup ur5e.launch.py ur_type:=ur5e robot_ip:=192.168.1.111
```

> Also install URCap and configure the robot IP address to `192.168.1.164`.

---

## Part 4: RG2 Gripper Control (ROS 2)

### Repo

[https://github.com/karishmapatnaik/onrobot_ros](https://github.com/karishmapatnaik/onrobot_ros)

### Run Gripper Control Node

```bash
ros2 run onrobot_ros gripper_control
```

### Send Gripper Commands

```bash
ros2 topic pub --once /gripper_command std_msgs/String "{data: '200'}"    # Position: 0–1100
ros2 topic pub --once /gripper_command std_msgs/String "{data: 'open'}"   # Fully open
ros2 topic pub --once /gripper_command std_msgs/String "{data: 'close'}"  # Fully close
```

---

## Part 5: Jenga Pose Estimation (Hand-in-Eye)

For vision-based pose localization:

### Repo

[https://github.com/MaxlGao/max_camera_localizer](https://github.com/MaxlGao/max_camera_localizer)

### Run Localizer

```bash
ros2 run max_camera_localizer localize
```

> Use this to find pose of Jenga blocks using intelrealsense camera.

---