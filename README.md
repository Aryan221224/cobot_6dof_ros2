# 6-DOF Teleoperated Precision Cobot
### ROS2 Jazzy | URDF + RViz Simulation | Dual-Mode End Effector

**Designed for:** Articulus Surgical · Integra Robotics  
**Author:** Aryan | Bennett University | R&D Associate, Weber Innovation  
**Stack:** ROS2 Jazzy · Xacro · robot_state_publisher · ros2_control · RViz2

---

## Overview

A complete ROS2 simulation of a 6-DOF serial cobot designed for two domains:

- **Surgical robotics** — Impedance-controlled teleoperation with non-homothetic motion scaling
- **Lab automation** — Motorised pipette module for precision liquid handling

The arm features a **dual-mode end effector**: a 2-finger parallel gripper and a motorised pipette, both on a quick-swap mount at the tool flange.

---

## Project Specifications

| Parameter | Value |
|-----------|-------|
| DOF | 6 (6R serial) |
| Reach | ~800 mm spherical |
| Payload | 1–2 kg |
| Repeatability | ±1 mm |
| End effector | Parallel gripper + pipette (dual-mode) |
| Control | Position (sim) → Impedance (hardware) |
| Software | ROS2 Jazzy, Xacro, ros2_control |

### Joint Configuration

| Joint | Axis | Function | Drive | Limit |
|-------|------|----------|-------|-------|
| J1 | Z | Base rotation | BLDC + Harmonic | ±180° |
| J2 | Y | Shoulder lift | BLDC + Harmonic | ±135° |
| J3 | Y | Elbow | BLDC + Harmonic | ±150° |
| J4 | X | Wrist roll | Mini servo | ±180° |
| J5 | Y | Wrist pitch | Mini servo | ±120° |
| J6 | Z | Tool rotation | Mini servo | ±180° |

---

## Quick Start

### Prerequisites

- Ubuntu 24.04 (or WSL2 on Windows)
- Internet connection for ROS2 install (~800 MB)

### 1. Clone the repository

```bash
git clone https://github.com/Aryan221224/cobot_6dof_ros2.git cobot_ws
cd cobot_ws
```

### 2. Run the one-shot setup script

```bash
chmod +x setup_cobot_env.sh
./setup_cobot_env.sh
```

### 3. Reload your shell

```bash
source ~/.bashrc
```

### 4. Launch the visualisation

```bash
# Interactive mode — sliders for each joint:
ros2 launch cobot_description display.launch.py

# Automated demo sequence:
ros2 launch cobot_description demo_motion.launch.py

# Full system (impedance + vision):
ros2 launch cobot_description full_system.launch.py
```

---

## Package Structure

```
cobot_ws/
├── setup_cobot_env.sh          ← One-shot installer
├── DEMO_VIDEO_SCRIPT.md        ← Video narration + LinkedIn caption
└── src/
    └── cobot_description/
        ├── package.xml
        ├── CMakeLists.txt
        ├── setup.py
        ├── urdf/
        │   ├── cobot.urdf.xacro       ← Main robot URDF (parametric)
        │   ├── materials.xacro        ← Colour definitions
        │   └── end_effector.xacro     ← Gripper + pipette assembly
        ├── launch/
        │   ├── display.launch.py      ← Interactive RViz launch
        │   ├── demo_motion.launch.py  ← Automated demo launch
        │   └── full_system.launch.py  ← Full system launch
        ├── rviz/
        │   └── cobot_view.rviz        ← Pre-configured RViz config
        ├── config/
        │   └── cobot_controllers.yaml ← ros2_control config
        └── cobot_description/
            ├── __init__.py
            ├── demo_motion.py         ← Demo trajectory publisher
            ├── impedance_controller.py← 3-layer cascaded controller
            ├── teleop_node.py         ← Teleoperation input node
            └── vision_node.py         ← Vision perception node
```

---

## Technical Details

### URDF Architecture

Built in Xacro for parametric design. Key link lengths:

```
L1 (shoulder)  = 200 mm
L2 (upper arm) = 300 mm
L3 (forearm)   = 250 mm
L4 (wrist 1)   = 100 mm
L5 (wrist 2)   =  90 mm
L6 (flange)    =  70 mm
TCP offset      = 130 mm
```

### Control Architecture

- **Simulation:** mock_components/GenericSystem plugin for perfect position control
- **Hardware:** Replace plugin with STM32/CAN interface — controller YAML unchanged

### Demo Motion Node

Publishes to `/joint_states` at 50 Hz with cubic ease-in/ease-out interpolation:

1. Home position → Full reach → J1 sweep
2. Pick-and-place approach → Gripper open/close
3. Surgical precision posture → Wrist articulation
4. Pipette descent → Aspirate/dispense → Home (loops)

---

## Extending This Project

```bash
# Add MoveIt2 planning
sudo apt install ros-jazzy-moveit

# Add Gazebo simulation
sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-ros2-control

# Add teleoperation
sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy
```

---

## WSL2 Display Setup

### Windows 11 (WSLg)
No extra config needed — RViz2 launches directly.

### Windows 10 (VcXsrv)
```bash
export DISPLAY=:0
export LIBGL_ALWAYS_INDIRECT=1
```

---

## Licence

MIT — free to use, fork, and extend with attribution.

---

*Built with ROS2 Jazzy · Ubuntu 24.04 · Xacro · rviz2 · ros2_control*
