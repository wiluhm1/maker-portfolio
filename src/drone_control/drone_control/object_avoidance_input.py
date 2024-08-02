import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand
import time
from std_msgs.msg import Bool

import threading

class HoveringNode(Node):
    def __init__(self):
        super().__init__("object_avoidance")
        print("Self Destruct Sequence Initiated.")

        self.counter = 0
        self.armed = False

        self.bool_publisher = self.create_publisher(Bool, 'close_or_not', 10)

        self.trajectoryPublisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        print("Trajectory Publisher Created.")
        #self.target = [0, 0, 0]
        self.vehicleCommandPublisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        print("Vehicle Command Publisher Created.")
        self.offboardPublisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)

        self.create_timer(0.1, self.timercallback)
        print("Timing Sequence Initiated.")

        self.user_input_thread = threading.Thread(target=self.promptTarget)
        print("Secondary Timing Sequence Initiated. Core Meltdown Inevitable.")

        self.user_input_thread.daemon = True
        self.user_input_thread.start()

            
        self.center_d = 0.0
        self.left_d = 0.0
        self.right_d = 0.0
        self.top_d = 0.0
        self.bottom_d = 0.0
        self.altitude_bool = True 
        self.xcord = 0.0   
        self.ycord = 0.0
        self.zcord = 0.0
        self.target = [0, 0, -.25]



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
        setpoint.position = [float(self.target[0]), float(self.target[1]), float(self.target[2])]
        setpoint.yaw = 0.0
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
        increment = 0.05    # Change to preferred value
        height = 2.9        # in meters
        min_distance = 1    # Change to preferred value

        while rclpy.ok():
            self.top_d = float(input("Input distance from top: "))
            self.bottom_d = float(input("Input distance from bottom: "))
            self.left_d = float(input("Input distance from left: "))
            self.right_d = float(input("Input distance from right: "))
            self.center_d = float(input("Input distance from center: "))

            if self.altitude_bool:
                if self.top_d <= min_distance:
                    print("Obstacle detected ABOVE")
                    self.zcord -= increment
                    print("Moving DOWN")

                elif self.bottom_d <= min_distance:
                    print("Obstacle detected BELOW")
                    self.zcord += increment
                    print("Moving UP")

                else:
                    print("No vertical obstacles, checking horizontal direction...")
                    if self.zcord > 0.5 * height:
                        print(f"Current altitude: {self.zcord}m, moving DOWN")
                        self.zcord -= increment
                    else:
                        print(f"Current altitude: {self.zcord}m, moving UP")
                        self.zcord += increment

                self.target = [self.xcord, self.ycord, -1 * self.zcord]
                self.trajectorypublish()

                # Wrap the boolean in a Bool message and publish it
                bool_msg = Bool(data=self.altitude_bool)
                self.bool_publisher.publish(bool_msg)


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






