import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class ArmDisarmNode(Node):

    def __init__(self):
        super().__init__('arm_disarm_node')
        self.publisher_ = self.create_publisher(Bool, 'ArmDisarm', 10)
        self.toggle_service_ = self.create_service(Trigger, 'toggle_arm_disarm', self.toggle_callback)
        self.state = False  # Initial state: Disarm
        self.get_logger().info('Node initialized and ready to toggle Arm/Disarm')

    def toggle_callback(self, request, response):
        self.state = not self.state
        self.publish_state()
        response.success = True
        response.message = f'State toggled to: {self.state}'
        return response

    def publish_state(self):
        msg = Bool()
        msg.data = self.state
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published state: {self.state}')


def main(args=None):
    rclpy.init(args=args)
    node = ArmDisarmNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
