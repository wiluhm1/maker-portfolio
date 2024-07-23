import rclpy
from rclpy.node import Node
import numpy as np

class OpenLoopControl(Node):
    status = False #True = armed, False = disarmed

    def __init__(self):
        super().__init__("open_loop_subscriber")
        self.subscription = self.create_subscription(#TODO, TODO, self.open_loop_callback, 10) # message type, topic name, callback function, queue size

    def open_loop_callback(self, msg):
        globals()['status'] = not status
        return status

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopControl()
    rclpy.spin(node)
    rclpy.shutdown()
