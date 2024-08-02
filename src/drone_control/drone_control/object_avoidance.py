import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class ObjectAvoidance(Node):

    def __init__(self):
        super().__init__("object_avoidance")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10)

        # message type, topic name, callback function, queue size
        self.real_position = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.position_callback, qos_profile)

        self.center_sub = self.create_subscription(Float32, 'roi_center_distance', self.center_callback, 10)
        self.left_sub = self.create_subscription(Float32, 'roi_left_distance', self.left_callback, 10)
        self.right_sub = self.create_subscription(Float32, 'roi_right_distance', self.right_callback, 10)
        self.top_sub = self.create_subscription(Float32, 'roi_top_distance', self.top_callback, 10)
        self.bottom_sub = self.create_subscription(Float32, 'roi_bottom_distance', self.bottom_callback, 10)
        
        self.bool_publisher = self.create_publisher(Bool, 'close_or_not', 10)
        self.trajectoryPublisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        #publish if or if not too close
    
        self.center_d = None
        self.left_d = None
        self.right_d = None
        self.top_d = None
        self.bottom_d = None
        self.altitude_bool = True 
        self.xcord = 0.0   
        self.ycord = 0.0
        self.zcord = 0.0
        self.target = [0.0, 0.0, 0.0]

        self.get_logger().info('Object Avoidance Started')


    def altitude_callback(self, msg):
        min_height = 0.3               #meters CHANGE to preferred value
        max_height = 2.9 - 0.8         #meters CHANGE to preferred value

        self.xcord = self.msg.position[0]
        self.ycord = self.msg.position[1]
        self.zcord = self.msg.position[2]

        if not (min_height <= self.zcord <= max_height):
            self.altitude_bool = False
            
    def center_callback(self, msg):
        self.center_d = msg.data
        self.process_detections()

    def left_callback(self, msg):
        self.left_d = msg.data
        self.process_detections()

    def right_callback(self, msg):
        self.right_d = msg.data
        self.process_detections()

    def top_callback(self, msg):
        self.top_d = msg.data
        self.process_detections()

    def bottom_callback(self, msg):
        self.bottom_d = msg.data
        self.process_detections()

    def trajectorypublish(self):
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(int(self.get_clock().now().to_msg().sec) * 1e6)
        setpoint.position = [float(self.target[0]), float(self.target[1]), float(self.target[2])]
        setpoint.yaw = 0.0
        self.trajectoryPublisher.publish(setpoint)

    def process_detections(self):
        increment = 0.05    #CHANGE to preferred value
        height = 2.9        #in meters
        min_distance = 1    #CHANGE to preferred value

        if self.altitude_bool:
            if self.top_d <= min_distance:
                print("Something ABOVE")
                #move down
                self.zcord -= increment
                print("Moving DOWN")

            elif self.bottom_d <= min_distance:
                print("Something BELOW")
                #move up
                self.zcord += increment
                print("Moving UP")

            else:
                print("Horizontal blocked")
                if self.zcord > 0.5 * height:
                    print(f"{self.altitude}m, moving DOWN")
                    self.zcord -= increment

                else:
                    print(f"{self.altitude}m, moving UP")
                    self.zcord += increment
            self.target = [self.xcord, self.ycord, self.zcord]
            self.trajectorypublish()
            self.bool_publisher.publish(self.altitude_bool)

        
def main(args=None):    
    rclpy.init(args=args)
    node = ObjectAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
