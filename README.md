# RE-RASSOR Arm Simulation - Pixi

This repository contains the ROS 2 Humble simulation for the RE-RASSOR arm, converted to use **Pixi** for dependency management. This setup ensures a reproducible environment across different systems (Linux, macOS, etc.) without requiring system-wide ROS installations.

### Why Pixi?

* **[Reproducible Package Management for Robotics](https://prefix.dev/blog/reproducible-package-management-for-robotics)**  

## Prerequisites
* **Pixi** (Package Manager)
```bash
curl -fsSL https://pixi.sh/install.sh | sh
# or 
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
cd ros2_ws
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
cd ros2_ws
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
cd ros2_ws
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
cd ros2_ws
source install/setup.bash
ros2 run aruco_recognition aruco_pose_estimation.py
```

---
## Troubleshooting Installation
Failing at Colcon Build: Delete build artifacts and rebuild
```bash
rm -rf build install log
colcon build
```

Make sure you are running everything in Bash Shell
```bash
bash
```

# Running the Simulation
## Moving the Arm (MoveIt)
Go to the Rviz window (the one showing the orange robot).

Locate the MotionPlanning panel (usually bottom left).

Ensure "Planning Scene" is checked in the Displays panel.

Use the interactive marker (arrows/rings at the end of the orange arm) to drag the "ghost" arm to a new goal position.

Click the Plan & Execute button in the MotionPlanning panel.

The real arm in Gazebo will move to match the plan.
## Troubleshooting Simulation
Arm is slumped/limp: You likely forgot to run the commands in Terminal 2. The controllers must be loaded manually.

"Address already in use" error: An old simulation is still running in the background. Run killall -9 gzserver gzclient to force close it.

Camera not showing in Rviz: Click "Add" in the Displays panel, select "Camera", and set the Topic to /camera/image_raw.

Build fails: Ensure you are in the root ~/ros2_ws folder and have sourced /opt/ros/humble/setup.bash.

## Project Structure
pixi.toml: The dependency manifest. Defines the environment packages.

colcon_defaults.yaml: Default arguments for the build system.

ros2_ws/: The standard ROS 2 workspace structure.

