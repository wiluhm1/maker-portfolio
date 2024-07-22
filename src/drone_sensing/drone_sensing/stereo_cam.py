import rclpy
from rclpy.node import Node
import cv2
import depthai as dai
import numpy as np
import math
import time

class StereoCam(Node):

    def __init__(self):
        super().__init__('stereo_cam')
        self.stereo_cam_pub = self.create_publisher( #TODO, #TODO, 10))  # message type, topic name, queue size
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.send) #TODO update name

    def send(self): #TODO rename
        msg = String() #TODO message type
        msg.data = #TODO % self.i
        self.stereo_cam_pub.publish(msg)
        self.get_logger().info('Publsyishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = StereoCam()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()