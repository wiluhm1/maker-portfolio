import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand, VehicleOdometry
import time
import threading
#from stereo_cam import

class HoveringNode(Node):
    def __init__(self):
        super().__init__("hover_node")
        print("Self Destruct Sequence Initiated.")

        self.counter = 0

        self.trajectoryPublisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        print("Trajectory Publisher Created.")
        self.target = [0, 0, 0]
        self.yaw = float("nan")
        self.yawspeed = 1
        self.vehicleCommandPublisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        print("Vehicle Command Publisher Created.")
        self.offboardPublisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.subscription = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, 10)
        self.starting_height = 1000
        self.current_height = 0
        print("Getting poses")

        self.create_timer(0.1, self.timercallback)

        print("Timing Sequence Initiated.")

        self.user_input_thread = threading.Thread(target=self.promptTarget)
        print("Secondary Timing Sequence Initiated. Core Meltdown Inevitable.")

        self.user_input_thread.daemon = True
        self.user_input_thread.start()

    def odometry_callback(self, msg):
        self.current_height = msg.position[2]
        if self.starting_height == 1000:
            self.starting_height = self.current_height
    
    def timercallback(self):
        if self.counter == 10:
            self.vehiclecommandpublish(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self.arm()
        self.offboardpublish()
        self.trajectorypublish()

        if self.counter < 11:
            self.counter = self.counter + 1

    def offboardpublish(self):
        mode = OffboardControlMode()
        mode.timestamp = int(int(self.get_clock().now().to_msg().sec) * 1e6)
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        self.offboardPublisher.publish(mode)

    def trajectorypublish(self):
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(int(self.get_clock().now().to_msg().sec) * 1e6)
        setpoint.position = [float(self.target[0]), float(self.target[1]), float(self.target[2]) + float(self.starting_height)]
        setpoint.yaw = float("nan")
        setpoint.yawspeed = self.yawspeed
        self.trajectoryPublisher.publish(setpoint)

    def vehiclecommandpublish(self, command, param1, param2=0.0):
        vC = VehicleCommand()
        vC.command = command
        vC.param1 = float(param1)
        vC.param2 = float(param2)
        vC.target_system = 1
        vC.target_component = 1
        vC.source_system = 1
        vC.source_component = 1
        vC.from_external = True
        vC.confirmation = 0
        self.vehicleCommandPublisher.publish(vC)

    def arm(self):
        self.vehiclecommandpublish(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        print("Arming Sequence Initiated.")

    def disarm(self):
        self.vehiclecommandpublish(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        print("Warp Core Breach in T minus 7 Seconds.")

    def promptTarget(self):
        while rclpy.ok():
            xcord = input("What x coordinate do you want to go to? ")
            ycord = input("What y coordinate do you want to go to? ")
            zcord = input("What z coordinate do you want to go to? ")

            xcord = float(xcord)
            ycord = float(ycord)
            zcord = float(zcord) * -1

            self.target = [xcord, ycord, zcord]

            #This stuff might not work.
            landerspanders = input("Hey man, are you safely on the ground and wanting to disarm? (y/n)")
            if landerspanders == "y":
                self.disarm()
                rclpy.shutdown()
        
def main(args=None):    
    rclpy.init(args=args)
    node = HoveringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
