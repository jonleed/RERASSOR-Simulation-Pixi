import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'final_description'
    file_subpath = 'urdf/final.xacro'

   

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

#    load_joint_state_controller = ExecuteProcess(
 #   	cmd=['ros2','control','load_controller','--set-state','active','joint_state_broadcaster'],
  #  	output='screen',
   # 	shell=True
    #)
    
   # load_arm_controller = ExecuteProcess(
    #	cmd=['ros2','control','load_controller','--set-state','active','arm_controller'],
    #	output='screen',
    #	shell=True
    #)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot'],
                    output='screen')
                    

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}]
    )

    moveit_config = (
    	MoveItConfigsBuilder("final_description")
        .robot_description(file_path="urdf/final.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    use_sim_time = {"use_sim_time":True}
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            config_dict,
            {"use_sim_time": True}
            ],
    )
    
    rviz_config = os.path.join(os.path.join(get_package_share_directory("final_description"), "config"), "moveit.rviz")

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )
    
    ros2_controllers_path = os.path.join(
        get_package_share_directory("final_description_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    
    aruco_marker_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'aruco_marker',  # Name for the Aruco marker entity
            '-file', os.path.join(get_package_share_directory(pkg_name), 'models', 'aruco_block_long', 'model.sdf'),
            '-x', '0.35',  # X position (in meters)
            '-y', '0.35',  # Y position (in meters)
            '-z', '0.1',  # Z position (in meters)
            '-R', '0',  # Roll angle (in radians)
            '-P', '0',  # Pitch angle (in radians)
            '-Y', '-0.785'   # Yaw angle (in radians)
        ],
        output='screen'
    )
    

    # Run the node
    return LaunchDescription([
   # RegisterEventHandler(
    #		event_handler=OnProcessExit(
    #		    target_action = spawn_entity,
    #		    on_exit=[load_joint_state_controller],
    #		)
    #	),
    #	RegisterEventHandler(
    #	event_handler=OnProcessExit(
    #		    target_action = load_joint_state_controller,
    #		    on_exit=[load_arm_controller],
    #		)
    #	),
        gazebo,
        node_robot_state_publisher,
        joint_state_publisher_node,
        spawn_entity,
        run_move_group_node,
        rviz_node,
        aruco_marker_spawn_node,
    ])
