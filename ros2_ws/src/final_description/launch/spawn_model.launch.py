import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "final_description"  # Your package name
    model_name = "aruco_block_long"  # Change this to your model folder name
    
    # Construct the correct path to model.sdf
    model_path = os.path.join(
        get_package_share_directory(package_name), 
        "models", model_name, "model.sdf"
    )

    # Check if file exists
    if not os.path.exists(model_path):
        print(f"Error: model file not found at {model_path}")
    
    return LaunchDescription([
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", model_name,
                "-file", model_path,
                "-x", "0", "-y", "0", "-z", "0"
            ],
            output="screen"
        )
    ])

