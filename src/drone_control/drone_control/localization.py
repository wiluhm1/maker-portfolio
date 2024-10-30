import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray, Bool
from px4_msgs.msg import VehicleOdometry
import math

class goToOrigin(Node):
    def __init__(self):
        super().__init__("localization")
        self.counter = 0
        self.armed = False
        self.current_local_state = np.zeros((12, 1))
        self.global_state = [0, 0, 0]
        self.global_origin_in_local = [0, 0]
        self.x0, self.y0, self.z0 = 0, 0, 0 # last known local coordinates while over ar tag
        self.x1, self.y1, self.z1 = 0, 0, 0 # last known global coordinates while over ar tag
        self.local_origin_in_global = [0, 0]
        print('Initialized it all')
        
        self.ran = False
        self.ar_tag_subscriber = self.create_subscription(Float32MultiArray, "/ar_tag_data", self.localize, 10) ###
        ###self.detection_sub = self.create_subscriber(TagDetection, "/ar_tag_detections", self.detective, 10)
        self.avg_position = [0, 0, 0]
        self.yaw = 0
        self.odometry_data = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, 10)

        self.localized_publisher = self.create_publisher(Bool, '/localized', 10)
        self.global_coord_publisher = self.create_publisher(Float32MultiArray, '/global_coord', 10)
        self.local_of_origin_publisher = self.create_publisher(Float32MultiArray, '/local_of_origin', 10)
        self.global_of_origin_publisher = self.create_publisher(Float32MultiArray, '/global_of_origin', 10)
        self.local_to_global_ref_publisher = self.create_publisher(Float32, '/local_to_global_ref', 10)
        print('Prepared all publishers')

    ###def detective(self, msg):
    ###    self.avg_position[0] = msg.position.pose.position.x
    ###    self.avg_position[1] = msg.position.pose.position.y
    ###    self.avg_position[2] = msg.position.pose.position.z
        
    def odometry_callback(self, msg):
        # Update state with local poses
        print('we are locala with it')
        self.current_local_state[0, 0] = msg.position[0]
        self.current_local_state[1, 0] = msg.position[1]
        self.current_local_state[2, 0] = msg.position[2]

    def localize(self, msg):
        self.ar_tag_data = msg
        print('ar_tag_data: ' + str(self.ar_tag_data))
        print('global_state: ' + str(self.global_state))
        if self.ar_tag_data is None and self.global_state == [0, 0, 0]:
            #publish false to localized and don't publish anything anywhere else
            print('we\'re cooked we\'re cooked we\'re cooked')
            self.ran = False
        if self.ar_tag_data is not None:
            print('localiiiiiiiiized. with ar tag under yessirrr')
            # Updates global states if ar tag data exists
            self.global_state[0], self.global_state[1], self.global_state[2] = self.ar_tag_data[0], self.ar_tag_data[1], self.ar_tag_data[2]
            self.x1, self.y1, self.z1 = self.global_state[0], self.global_state[1], self.global_state[2]
            # Calculates current yaw
            self.yaw = -self.ar_tag_data[3]
            
            # Calculates global origin in local coordinates
            rotation_angle = -(self.ar_tag_data[3] + math.pi - math.atan2(self.ar_tag_data[0], self.ar_tag_data[1]))
            self.x0, self.y0, self.z0 = self.current_local_state[0, 0], self.current_local_state[1, 0], self.current_local_state[2, 0]
            total_dist = math.sqrt(self.global_state[0] ** 2 + self.global_state[1] ** 2)
            self.global_origin_in_local[0] = self.x0 + total_dist * math.cos(rotation_angle)
            self.global_origin_in_local[1] = self.y0 + total_dist * math.sin(rotation_angle)

            origin_to_origin_local = -1 * self.global_origin_in_local

            rotation_matrix = np.array([
                [math.cos(self.yaw), -math.sin(self.yaw)],
                [math.sin(self.yaw), math.cos(self.yaw)]
            ])
            self.local_origin_in_global = np.dot(rotation_matrix, origin_to_origin_local)
            self.ran = True
            print('We are at ' + str(self.global_state))
        
        if self.ar_tag_data is None and self.global_state is not None:
            print('no ar tag under but we tuff')
            self.global_state[2] = self.current_local_state[2, 0] - self.z0 + self.z1
            local_change = [self.current_local_state[0, 0] - self.x0, self.current_local_state[1, 0] - self.y0]
            rotation_matrix = np.array([
                [math.cos(self.yaw), -math.sin(self.yaw)],
                [math.sin(self.yaw), math.cos(self.yaw)]
            ])
            rotated_vector = np.dot(rotation_matrix, local_change)
            self.global_state[0], self.global_state[1] = self.x1 + rotated_vector[0], self.y1 + rotated_vector[1]
            origin_to_origin_local = -1 * self.global_origin_in_local
            self.local_origin_in_global = np.dot(rotation_matrix, origin_to_origin_local)
            self.ran = True
            print('We are at ' + str(self.global_state))
        
        self.local_of_origin_publisher.publish(self.global_origin_in_local)
        self.localized_publisher.publish(self.ran)
        self.global_coord_publisher.publish(self.global_state)
        self.local_to_global_ref_publisher.publish(self.yaw)
        print('published everything')



def main(args=None):    
    rclpy.init(args=args)
    node = goToOrigin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()