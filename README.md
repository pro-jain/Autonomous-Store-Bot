# Autonomous-Store-Bot
Autonomous store robot that maps indoor environments, identifies and stores object locations, and navigates to the nearest requested object using semantic commands.
# Semantic Autonomous Mobile Robot

An autonomous indoor mobile robot built using ROS2, Gazebo, YOLOv8, and Nav2 for semantic navigation and intelligent inventory mapping.

## Features

- Differential drive mobile robot built from scratch in ROS2
- Custom built gazebo simulation environment
- SLAM-based environment mapping
- Real-time object detection using YOLOv8
- Semantic inventory mapping with object coordinates
- Autonomous navigation using Nav2
- Obstacle avoidance and shortest-path planning
- Semantic goal commands like:
  - `goto laptop`
  - `goto nearest TV`
  - `find microwave`

---

## Tech Stack

- ROS2 Humble
- Gazebo
- Nav2
- YOLOv8
- OpenCV
- Python
- RViz2

---

## System Architecture

```text
Custom built Gazebo environment
    ↓
Camera Feed
    ↓
YOLOv8 Object Detection
    ↓
Semantic Object Mapping
    ↓
Coordinate Extraction
    ↓
Nav2 Goal Generation
    ↓
Autonomous Navigation
```

---

## Current Progress

### Completed

- Custom differential drive robot
- Custom built gazebo showroom environment
- Camera + LiDAR integration
- SLAM mapping
- YOLOv8 semantic detection
- Object coordinate storage
- Semantic command parsing
- Nav2 setup

### In Progress

- [ ] NLP command interface
- [ ] Web dashboard integration
- [ ] Remote robot control

---

## Example Commands

```bash
goto laptop
goto nearest tv
find microwave
```

---

## Folder Structure

```text
ws_ddmobile/
│
├── src/
│   ├── mobile_dd_robot/
│   ├── yolo_detector/
│   ├── semantic_navigation/
│
├── maps/
├── worlds/
├── models/
```

---

## Setup

### Clone Repository

```bash
git clone https://github.com/yourusername/semantic-autonomous-robot.git
cd semantic-autonomous-robot
```

### Build Workspace

```bash
colcon build
source install/setup.bash
```

### Launch Gazebo

```bash
ros2 launch mobile_dd_robot gazebo_model.launch.py
```

### Run SLAM

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

### Run YOLO Detector

```bash
ros2 run yolo_detector yolo_node
```

### Launch Nav2

```bash
ros2 launch nav2_bringup navigation_launch.py \
map:=~/my_map.yaml \
use_sim_time:=True
```

---

## Future Goals

- Voice-based navigation
- Multi-room semantic mapping
- Multi-robot coordination

---

## Author

Bodhini Jain
