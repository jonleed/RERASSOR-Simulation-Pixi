# RE-RASSOR Arm Simulation - Pixi

This repository contains the ROS 2 Humble simulation for the RE-RASSOR arm, converted to use **Pixi** for dependency management. This setup ensures a reproducible environment across different systems (Linux, macOS, etc.) without requiring system-wide ROS installations.

Read More about Pixi:

* **[Reproducible Package Management for Robotics](https://prefix.dev/blog/reproducible-package-management-for-robotics)** 

* **[RoboStack Documentation](https://robostack.github.io/GettingStarted.html)** 

## Prerequisites
* **Pixi** (Package Manager)
```bash
brew install pixi
```
---

## 1. Installation
Instead of manual apt commands, Pixi handles all ROS 2 Humble, Gazebo, and Python dependencies automatically in a local folder.
```bash
pixi install
```

## 2. Building the Project
All commands must be run inside the Pixi environment shell.

Enter the Pixi Shell:
```bash
pixi shell
```

Build the Workspace: We use colcon to build the ROS 2 packages.
```bash
colcon build
```

Source the Overlay: After building, you must source the installation to make the new packages visible.
```bash
source install/setup.bash
```

## 3. Running the Simulation
To launch the simulation, ensure you are inside the Pixi shell (pixi shell) and have sourced the setup file.

### Terminal 1: Launch the Environment
This starts the physics engine (Gazebo), visualization (RViz), and the robot state publisher.

```bash
# 1. Enter the environment 
pixi shell

# 2. Source the workspace
source install/setup.bash

# 3. Launch the bringup file
ros2 launch final_description bringup.launch.py
```

### Terminal 2: Activate Controllers
The robot may spawn in a passive state. You need to manually activate the ROS 2 controllers and arm motors.

```bash
# 1. Enter the environment
pixi shell

# 2. Source the workspace
source install/setup.bash

# 3. Load the Joint State Broadcaster (Reads joint positions)
ros2 control load_controller --set-state active joint_state_broadcaster

# 4. Load the Arm Controller (Activates motors)
ros2 control load_controller --set-state active arm_controller\
```
### Terminal 3 (Optional): Aruco Recognition
To run the computer vision node for Aruco tag detection:

```bash
pixi shell
source install/setup.bash
ros2 run aruco_recognition aruco_pose_estimation.py
```

---
# Troubleshooting
Failing at Colcon Build: Delete build artifacts and rebuild
```bash
rm -rf build install log
colcon build
```

## Project Structure
pixi.toml: The dependency manifest. Defines the environment packages.

colcon_defaults.yaml: Default arguments for the build system.

ros2_ws/: The standard ROS 2 workspace structure.

