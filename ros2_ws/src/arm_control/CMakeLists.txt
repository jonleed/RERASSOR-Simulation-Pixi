cmake_minimum_required(VERSION 3.5)
project(arm_control)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # ROS 2 Python client library

# Install Python scripts
install(PROGRAMS
  scripts/arm_control_with_kdl.py
  DESTINATION lib/${PROJECT_NAME}
)

# Mark package for ament
ament_package()

