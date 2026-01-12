#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
import numpy as np
from scipy.spatial.transform import Rotation as R

class ArmTest(Node):
    def __init__(self):
        super().__init__('arm_test')
        self.client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.client.wait_for_server()
        self.get_logger().info("Action server available.")
        
        # Subscribe to the block_pose topic from ArUco estimation
        self.create_subscription(Pose, '/block_pose', self.block_pose_callback, 10)
        self.block_pose = None

    def transform_to_world_frame(self, pose):
        # Camera origin and orientation in world frame
        camera_position = np.array([0.4, 0.6, 1.5])
        camera_rotation = R.from_euler('xyz', [-np.pi / 2, 0, -np.pi / 2])

        
        # Debug: Print rotation matrix
        self.get_logger().info(f"Camera Rotation Matrix:\n{camera_rotation.as_matrix()}")
        
        # Convert block pose to numpy array
        block_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        self.get_logger().info(f"Original Block Position (Camera Frame): {block_position}")
        
        # Transform block position from camera frame to world frame
        block_position_world = camera_rotation.apply(block_position) + camera_position
        self.get_logger().info(f"Transformed Block Position (World Frame): {block_position_world}")
        
        return block_position_world

    def block_pose_callback(self, msg):
        # Transform block pose to world frame
        transformed_position = self.transform_to_world_frame(msg)
        self.get_logger().info(f"Final Transformed Block Pose - x: {transformed_position[0]}, y: {transformed_position[1]}, z: {transformed_position[2]}")
        self.send_trajectory(transformed_position)

    def send_trajectory(self, target_position):
        x, y, z = target_position
        y += 1.0  # Move 10 cm above the block
        
        # Convert block position to joint angles (placeholder, use IK solver)
        joint_positions = [x, y, z, 0.0]
        
        if any(joint_position is None for joint_position in joint_positions):
            self.get_logger().error("Invalid joint positions!")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ['Revolute 40', 'Revolute 43', 'Revolute 44', 'Revolute 45']
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=2)
        goal.trajectory.points.append(point)

        self.get_logger().info("Sending trajectory...")
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.trajectory_callback)

    def trajectory_callback(self, future):
        result = future.result()
        if result.accepted:
            self.get_logger().info("Trajectory accepted and started execution.")
        else:
            self.get_logger().error("Trajectory was not accepted.")


def main(args=None):
    rclpy.init(args=args)
    arm_test = ArmTest()
    rclpy.spin(arm_test)

if __name__ == '__main__':
    main()
