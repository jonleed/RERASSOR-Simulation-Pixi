# RE-RASSOR Arm Simulation (Fixed Version)

This repository contains the fully functional ROS 2 simulation for the RE-RASSOR arm. It includes fixes for the original repository's build errors, controller failures ("slumped arm"), and missing dependencies.

## Prerequisites
* **OS:** Ubuntu 22.04 (Jammy Jellyfish)
* **Hardware:** Valid Virtual Machine or Native Ubuntu installation.

---

## 1. System Setup (One-Time Only)

If you have a fresh Ubuntu installation, you must install ROS 2 Humble first.

### 1.1 Set Locale
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### 1.2 Add ROS 2 Sources
```bash

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(. /etc/os-release && echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 1.3 Install ROS 2 Humble
```bash

sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
```
### 1.4 Environment Setup
Add the ROS 2 setup script to your .bashrc so you don't have to source it manually every time.

```bash

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### 2. Project Installation
### 2.1 Clone the Fixed Repository
Clone this repository directly into your home directory to act as your workspace (~/ros2_ws).

```bash

cd ~
git clone [https://github.com/evahocking/RERASSOR-Simulation.git](https://github.com/evahocking/RERASSOR-Simulation.git) ros2_ws
```
### 2.2 Install Simulation Dependencies (Critical)
You must install these specific packages. Without them, Gazebo will crash or the arm will be limp.

```bash

sudo apt-get update
sudo apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-configs-utils \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    python3-opencv \
    ros-humble-cv-bridge

```
### 2.3 Build the Workspace
Navigate to the workspace root and build.

```bash

cd ~/ros2_ws
colcon build
```
(Note: "Stderr" output during the build is normal as long as it says "Finished" at the end).

### 3. Running the Simulation
You must use two terminals to run the full simulation.

Terminal 1: Launch the Simulation Environment
This starts Gazebo (physics), Rviz (visualization), and the robot model.

```bash

cd ~/ros2_ws
source install/setup.bash
ros2 launch final_description bringup.launch.py
```
Note: The arm will spawn in a "slumped" state (limp) inside Gazebo. This is expected behavior until you run Terminal 2.

Terminal 2: Wake Up the Controllers
Open a new terminal to activate the motors.

```bash

cd ~/ros2_ws
source install/setup.bash

# 1. Load the joint state broadcaster (reads arm position)
ros2 control load_controller --set-state active joint_state_broadcaster

# 2. Load the arm controller (This makes the arm stand up!)
ros2 control load_controller --set-state active arm_controller
```
The moment you run the second command, the arm in Gazebo should snap to the upright position.

## this section only i have not tested - this is for future use if i get it running
Terminal 3 (Optional): Aruco Recognition
To run the computer vision node:

```bash

cd ~/ros2_ws
source install/setup.bash
ros2 run aruco_recognition aruco_pose_estimation.py
```
### 4. Moving the Arm (MoveIt)
Go to the Rviz window (the one showing the orange robot).

Locate the MotionPlanning panel (usually bottom left).

Ensure "Planning Scene" is checked in the Displays panel.

Use the interactive marker (arrows/rings at the end of the orange arm) to drag the "ghost" arm to a new goal position.

Click the Plan & Execute button in the MotionPlanning panel.

The real arm in Gazebo will move to match the plan.

### Troubleshooting
Arm is slumped/limp: You likely forgot to run the commands in Terminal 2. The controllers must be loaded manually.

"Address already in use" error: An old simulation is still running in the background. Run killall -9 gzserver gzclient to force close it.

Camera not showing in Rviz: Click "Add" in the Displays panel, select "Camera", and set the Topic to /camera/image_raw.

Build fails: Ensure you are in the root ~/ros2_ws folder and have sourced /opt/ros/humble/setup.bash.
