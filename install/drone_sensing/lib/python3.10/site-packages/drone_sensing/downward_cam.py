'''
import rclpy
from rclpy.node import Node

class DownwardCam(Node):

    def __init__(self):
        super().__init__('downward_cam')
        self.downward_cam_pub = self.create_publisher( #TODO, #TODO, 10))  # message type, topic name, queue size
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.send) #TODO update name

    def send(self): #TODO rename
        msg = String() #TODO message type
        msg.data = #TODO % self.i
        self.downward_cam_pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = DownwardCam()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


'''


# ___Import Modules:
import os
import cv2
import json
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
#from ament_index_python.packages import get_package_share_directory

# ___Global Variables:
DEVICE_INDEX = 0  # specifies which camera
TOPIC = '/rosout'
QUEUE_SIZE = 1
PERIOD = 0.1  # seconds


# __Classes:
class CameraPublisher(Node):
    """Camera Publisher Class.

    This class contains all methods to publish camera data and info in ROS 2
    topic in sensor_msgs.msg/Image format.

    """

    def __init__(self, capture, topic=TOPIC, queue=QUEUE_SIZE, period=PERIOD):
        """Constructor.

        Args:
            capture: OpenCV Videocapture object.

        """

        super().__init__('camera_publisher')

        # initialize publisher
        self.publisher_ = self.create_publisher(Image, globals()['TOPIC'], globals()['QUEUE_SIZE'])
        timer_period = globals()['PERIOD']
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # set image counter and videocapture object
        self.capture = capture
        self.i = 0

    def timer_callback(self):
        """Timer Callback Function

        This method captures images and publishes required data in ros 2 topic.

        """

        if self.capture.isOpened():
            # reads image data
            ret, frame = self.capture.read()

            # processes image data and converts to ros 2 message
            msg = Image()
            msg.header.stamp = Node.get_clock(self).now().to_msg()
            msg.header.frame_id = 'ANI717'
            msg.height = np.shape(frame)[0]
            msg.width = np.shape(frame)[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = False
            msg.step = np.shape(frame)[2] * np.shape(frame)[1]
            msg.data = np.array(frame).tobytes()

            # publishes message
            self.publisher_.publish(msg)
            self.get_logger().info('%d Images Published' % self.i)

        # image counter increment
        self.i += 1

        return None


# ___Main Method:
def main(args=None):
    """This is the Main Method.

    """

    # loads setting file set parameters
    #settings = os.path.join(get_package_share_directory('ros2_camera_publish'),
                            #"settings.json")

    #with open(settings) as fp:
        #content = json.load(fp)

        # creates OpenCV Videocapture object
    capture = cv2.VideoCapture(globals()['DEVICE_INDEX'])
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)

    # initializes node and start publishing
    rclpy.init(args=args)
    camera_publisher = CameraPublisher(capture, globals()['TOPIC'],
                                       globals()['QUEUE_SIZE'],
                                       globals()['PERIOD'])
    rclpy.spin(camera_publisher)

    # shuts down nose and releases everything
    camera_publisher.destroy_node()
    rclpy.shutdown()
    capture.release()

    return None


# ___Driver Program:
if __name__ == '__main__':
    main()

