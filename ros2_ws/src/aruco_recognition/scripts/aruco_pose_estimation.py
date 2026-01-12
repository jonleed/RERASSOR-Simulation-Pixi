#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os, sys, math, time

from std_msgs.msg import String, Int32, Float64
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from math import pi

import numpy as np
import cv2
import cv2.aruco as aruco

from cv_bridge import CvBridge

bridge = CvBridge() 
arucoMarkerLength = 0.057 
font = cv2.FONT_HERSHEY_SIMPLEX

class ArucoRecognition(Node):
    def __init__(self):
        super().__init__('aruco_recognition')

        self.camera_matrix = np.matrix([[381.36246688113556, 0.0, 320.5], 
                                        [0.0, 381.36246688113556, 240.5], 
                                        [0.0, 0.0, 1.0]])
        self.distortion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        
        self.detector = aruco.ArucoDetector(self.dictionary, aruco.DetectorParameters())

        self.pub_block_pose = self.create_publisher(Pose, '/block_pose', 10)
        self.create_subscription(Image, "/camera/image_raw", self.callback_color_img, 10)
        
        self.get_logger().info("Aruco Recognition Node has been started")

    def find_ARMarker(self, frame):
        self.frame = frame
        self.corners, self.ids, _ = self.detector.detectMarkers(self.frame)
        if self.corners is None or len(self.corners) == 0:
            self.get_logger().warn("No ArUco markers detected")
        else:
            self.get_logger().info(f"Detected {len(self.corners)} marker(s)")
        aruco.drawDetectedMarkers(self.frame, self.corners, self.ids, (0,255,0))


    def get_ARMarker_pose(self, i):
        if self.corners is not None and len(self.corners) > 0 and i < len(self.corners):
            marker_corners = self.corners[i][0]
            if len(marker_corners) >= 4:
                obj_points = np.array([
                    [-arucoMarkerLength / 2, -arucoMarkerLength / 2, 0],  # Bottom-left
                    [arucoMarkerLength / 2, -arucoMarkerLength / 2, 0],   # Bottom-right
                    [arucoMarkerLength / 2, arucoMarkerLength / 2, 0],    # Top-right
                    [-arucoMarkerLength / 2, arucoMarkerLength / 2, 0]    # Top-left
                ], dtype=np.float32)
                
                image_points = np.array(marker_corners, dtype=np.float32).reshape(-1, 1, 2)
                
                try:
                    
                    success, rvec, tvec = cv2.solvePnP(obj_points, image_points, self.camera_matrix, self.distortion)
                    
                    if not success:
                        self.get_logger().error("cv2.solvePnP did not return a valid result.")
                        return None, None
                    
                    
                    if rvec is None or tvec is None:
                        self.get_logger().error("rvec or tvec is None, pose estimation failed.")
                        return None, None
                    if rvec.shape != (3, 1) and rvec.shape != (1, 3):
                        self.get_logger().error(f"Invalid shape for rvec: {rvec.shape}")
                    if tvec.shape != (3, 1):
                        self.get_logger().error(f"Invalid shape for tvec: {tvec.shape}")
                    
                    if rvec.shape == (3, 1):
                        self.frame = cv2.drawFrameAxes(self.frame, self.camera_matrix, self.distortion, rvec, tvec, 0.1)
                    return rvec, tvec
                except Exception as e:
                    self.get_logger().error(f"Error in estimatePoseSingleMarkers: {e}")
            else:
                self.get_logger().warn(f"Not enough corners detected: {len(marker_corners)} corners")
        else:
            self.get_logger().warn(f"No valid corners found or invalid index: {i}")
                
        return None, None

    def get_degrees(self, i):
        rvec, tvec = self.get_ARMarker_pose(i)

        if rvec is not None:

            Xtemp = tvec[0][0]
            Ytemp = tvec[1][0] * math.cos(-pi/4) - tvec[2][0] * math.sin(-pi/4)
            Ztemp = tvec[1][0] * math.sin(-pi/4) + tvec[2][0] * math.cos(-pi/4)
            
            Xtarget = - (Xtemp - 0.4)
            Ytarget = - (Ztemp - 0.5)
            Ztarget = - (Ytemp - 0.4)
            
            roll_angle, pitch_angle, yaw_angle = rvec[0][0]*180/pi, rvec[1][0]*180/pi, rvec[2][0]*180/pi
            
            if pitch_angle < 0:
                roll_angle, pitch_angle, yaw_angle = -roll_angle, -pitch_angle, -yaw_angle
            
            self.get_logger().info(f"(X, Y, Z) : {Xtarget}, {Ytarget}, {Ztarget}")
            return Xtarget, Ytarget, Ztarget, roll_angle, pitch_angle, yaw_angle
        
        return None

    def callback_color_img(self, data):
        cv_color_image = bridge.imgmsg_to_cv2(data, "bgr8")
        self.find_ARMarker(cv_color_image)
        
        try:
            result = self.get_degrees(0)
            if result:
                Xtarget, Ytarget, Ztarget, roll_angle, pitch_angle, yaw_angle = result
                block_pose = Pose()
                block_pose.position.x = Xtarget
                block_pose.position.y = Ytarget
                block_pose.position.z = Ztarget
                self.pub_block_pose.publish(block_pose)
        except:
            self.get_logger().warn("No marker detected!")

        cv2.imshow("result", self.frame)
        if cv2.waitKey(1) > 0:
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoRecognition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

