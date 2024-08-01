#!/usr/bin/env python3

# Every frame an AR tag is detected, this node will publish the calculated 3D coordinates (meters) and yaw (radians) of the camera in a list: [x, y, z, yaw angle]

import rclpy
from rclpy.node import Node
import cv2 as cv
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import math

class ARTagsDetectNode(Node):

    br_: CvBridge
    LIBRARY_: cv.aruco.Dictionary
    PARAMETERS_ = cv.aruco.DetectorParameters
    DETECTOR_: cv.aruco.ArucoDetector

    def __init__(self):
        super().__init__("camera_read_ar_tags")

        self.image_subscriber = self.create_subscription(Image, "/image_raw", self.publish_ar_tag_data, 10)
        self.publisher = self.create_publisher(Float32MultiArray, "/ar_tag_data", 10)

        self.br_ = CvBridge()

        self.LIBRARY_ = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_100)
        self.PARAMETERS_ = cv.aruco.DetectorParameters()
        self.DETECTOR_ = cv.aruco.ArucoDetector(self.LIBRARY_, self.PARAMETERS_)

        self.get_logger().info("AR tags node sucessfully launched!")

    def publish_ar_tag_data(self, image: Image):
        img = self.br_.imgmsg_to_cv2(image)

        tag_corners, tag_IDs, unused_rejected_candidates = self.DETECTOR_.detectMarkers(cv.cvtColor(img, cv.COLOR_BGR2GRAY))

        # Challenge-specific AR tag information
        actual_locations = [[1, 0, 0, 0], [14, 2, 0, 0], [5, 3, 1, 0], [13, 1, 2, 0], [27, 0, 3, 0], [25, 2, 3, 0], [6, 4, 3, 0], [4, 1, 4, 0], [20, 3, 5, 0], [17, 0, 6, 0], [33, 2, 6, 0], [23, 4, 7, 0], [22, 0, 8, 0], [21, 2, 8, 0], [10, 4, 9, 0], [18, 1, 10, 0], [24, 3, 10, 0], [26, 2, 11, 0], [12, 4, 11, 0], [19, 0, 13, 0], [9, 3, 13, 0]]
        tag_side_length = 0.266 # Side length of tag in meters (26.6 cm according to Sakina)
        half_side = tag_side_length / 2

        # Convert coordinates of AR tags to meters, since they were originally in units of 60 cm for whatever reason lol
        for i in range(len(actual_locations)):
            for j in range(1, len(actual_locations[i])):
                actual_locations[i][j] = actual_locations[i][j] * 0.6

        if tag_IDs != None: # If one or more tags are detected...
            for tag in actual_locations:
                if tag[0] == tag_IDs[0]: # ...localize the position relative to the first one in the (arbitrarily ordered) list
                    corner1 = [tag[1] - half_side, tag[2] - half_side, 0]
                    corner2 = [tag[1] + half_side, tag[2] - half_side, 0]
                    corner3 = [tag[1] + half_side, tag[2] + half_side, 0]
                    corner4 = [tag[1] - half_side, tag[2] + half_side, 0]

                    # cv2.detectMarkers outputs tag_corners in counterclockwise order starting from bottom left
                    object_points = np.array([corner1, corner2, corner3, corner4])
                    image_points = np.array([tag_corners[0][0][0], tag_corners[0][0][1], tag_corners[0][0][2], tag_corners[0][0][3]])

                    # Intrinsic camera information from calibration script
                    camera_calibration_matrix = np.array([[1112.452737448037, 0.0, 292.2046303286278], [0.0, 1109.2955799630336, 199.53242054186512], [0.0, 0.0, 1.0]])
                    distortion_coefficients = np.array([[-0.02908171947553649, 1.9551162980971628, 0.0059748015106549585, 0.0009355439491551768, 0.0]])
                    
                    # Calculate translation and rotation vectors based on corresponding 3D and 2D coordinates
                    success, rotation_vector, translation_vector = cv.solvePnP(object_points, image_points, camera_calibration_matrix, distortion_coefficients)
                    camera_pose = translation_vector.flatten().tolist()

                    # Correcting random sign errors introduced during the calculation
                    camera_pose[0] = -1 * camera_pose[0]
                    rotation_vector[2] = -1 * rotation_vector[2]

                    # Negating z-axis camera rotation so x and y coordinates in camera_pose are independent of yaw and represent the drone's actual position
                    x, y, yaw = camera_pose[0], camera_pose[1], -rotation_vector[2]
                    camera_pose[0] = (x * math.cos(yaw)) - (y * math.sin(yaw))
                    camera_pose[1] = (x * math.sin(yaw)) + (y * math.cos(yaw))

                    # Continuously print data if necessary
                    """
                    for i in range(len(camera_pose)):
                        self.get_logger().info(f"Pose index {i}: {camera_pose[i]}")
                    for i in range(len(rotation_vector)):
                        self.get_logger().info(f"Rotation index {i}: {rotation_vector[i]}")
                    """
                    
                    # Publishing time
                    msg = Float32MultiArray()
                    output_array = []
                    msg.data = [float(camera_pose[0]), float(camera_pose[1]), float(camera_pose[2]), float(rotation_vector[2])]
                    self.publisher.publish(msg)

def main(args=None):
    
    rclpy.init(args=args)

    # Create a node
    node = ARTagsDetectNode()
    rclpy.spin(node)

    # Shutdown the rclpy
    rclpy.shutdown()


if __name__ == "__main__":
    main()
