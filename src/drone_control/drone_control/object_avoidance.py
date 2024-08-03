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

        self.sub_1 = self.create_subscription(Float32, 'roi_1_distance', self.callback_1, 10)
        self.sub_2 = self.create_subscription(Float32, 'roi_2_distance', self.callback_2, 10)
        self.sub_3 = self.create_subscription(Float32, 'roi_3_distance', self.callback_3, 10)
        self.sub_4 = self.create_subscription(Float32, 'roi_4_distance', self.callback_4, 10)
        self.sub_5 = self.create_subscription(Float32, 'roi_5_distance', self.callback_5, 10)
        self.sub_6 = self.create_subscription(Float32, 'roi_6_distance', self.callback_6, 10)
        self.sub_7 = self.create_subscription(Float32, 'roi_7_distance', self.callback_7, 10)
        self.sub_8 = self.create_subscription(Float32, 'roi_8_distance', self.callback_8, 10)
        self.sub_9 = self.create_subscription(Float32, 'roi_9_distance', self.callback_9, 10)

        self.bool_publisher = self.create_publisher(Bool, 'close_or_not', 10)
        self.trajectoryPublisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        #publish if or if not too close
    
        self.d1 = 0.0
        self.d2 = 0.0
        self.d3 = 0.0
        self.d4 = 0.0
        self.d5 = 0.0
        self.d6 = 0.0
        self.d7 = 0.0
        self.d8 = 0.0
        self.d9 = 0.0
        self.altitude_bool = True 
        self.xcord = 0.0   
        self.ycord = 0.0
        self.zcord = 0.0
        self.target = [0.0, 0.0, 0.0]

        self.get_logger().info('Object Avoidance Started')


    def position_callback(self, msg):
        min_height = 0.3               #meters CHANGE to preferred value
        max_height = 2.9 - 0.8         #meters CHANGE to preferred value

        self.xcord = self.msg.position[0]
        self.ycord = self.msg.position[1]
        self.zcord = self.msg.position[2]

        if not (min_height <= self.zcord <= max_height):
            self.altitude_bool = False
            
    def callback_1(self, msg):
        self.d1 = msg.data
        self.process_detections()

    def callback_2(self, msg):
        self.d2 = msg.data
        self.process_detections()

    def callback_3(self, msg):
        self.d3 = msg.data
        self.process_detections()

    def callback_4(self, msg):
        self.d4 = msg.data
        self.process_detections()

    def callback_5(self, msg):
        self.d5 = msg.data
        self.process_detections()

    def callback_6(self, msg):
        self.d6 = msg.data
        self.process_detections()

    def callback_7(self, msg):
        self.d7 = msg.data
        self.process_detections()

    def callback_8(self, msg):
        self.d8 = msg.data
        self.process_detections()

    def callback_9(self, msg):
        self.d9 = msg.data
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
        up_counter = 0
        down_counter = 0
        middle_counter = 0

        if self.altitude_bool:
            down_counter = sum(1 for d in {self.d1, self.d2, self.d3} if d <= min_distance)
            up_counter = sum(1 for d in {self.d7, self.d8, self.d9} if d <= min_distance)
            middle_counter = sum(1 for d in {self.d4, self.d5, self.d6} if d <= min_distance)

            if down_counter > up_counter and down_counter > middle_counter:
                print("Something ABOVE")
                #move down
                self.zcord -= increment
                print("Moving DOWN")

            elif up_counter > down_counter and up_counter > middle_counter:
                print("Something BELOW")
                #move up
                self.zcord += increment
                print("Moving UP")

            else:
                print("Referencing altitude...")
                if self.zcord > 0.5 * height:
                    print(f"{self.zcord}m, moving DOWN")
                    self.zcord -= increment
                else:
                    print(f"{self.zcord}m, moving UP")
                    self.zcord += increment
            self.target = [self.xcord, self.ycord, self.zcord]
            self.trajectorypublish()
            bool_msg = Bool(data=self.altitude_bool)
            self.bool_publisher.publish(bool_msg)

        
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
