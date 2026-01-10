#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool
from smartec_msgs.msg import SmartecStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import can
import cantools
import math


class SmartecDriver(Node, can.Listener):

    def __init__(self):
        super().__init__("smartec_motor_driver")

        self.last_cmd_time = self.get_clock().now()
        self.last_odom_time = self.get_clock().now()
        self.left_status = SmartecStatus()
        self.right_status = SmartecStatus()
        self.pose = [0, 0, 0]
        self.teleop_switch = False

        self.setup_parameters()
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.init_subscribers()
        self.init_publishers()
        self.init_timers()
        self.init_canbus()

        self.stop_motors()

    def setup_parameters(self):
        self.declare_parameter("interface", "socketcan")
        self.declare_parameter("channel", "can0")
        self.declare_parameter("bitrate", 500000)
        self.declare_parameter("database_file", "")
        self.declare_parameter("command_rate", 50)
        self.declare_parameter("status_rate", 20)
        self.declare_parameter("odom_rate", 50)
        self.declare_parameter("base_width", 1.0)
        self.declare_parameter("wheel_radius", 0.1825)
        self.declare_parameter("gear_ratio", 10.85)
        self.declare_parameter("idle_timeout", 0.5)
        self.declare_parameter("debug", False)

        # These modes match the names give by HydroGear
        self.declare_parameter("rpm_control_mode", 2)
        self.declare_parameter("hard_stop_mode", 1)

        self.interface = self.get_parameter("interface").value
        self.channel = self.get_parameter("channel").value
        self.bitrate = self.get_parameter("bitrate").value
        self.database_file = self.get_parameter("database_file").value
        self.command_rate = self.get_parameter("command_rate").value
        self.status_rate = self.get_parameter("status_rate").value
        self.odom_rate = self.get_parameter("odom_rate").value
        self.base_width = self.get_parameter("base_width").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.gear_ratio = self.get_parameter("gear_ratio").value
        self.idle_timeout = self.get_parameter("idle_timeout").value
        self.debug = self.get_parameter("debug").value
        self.rpm_control_mode = self.get_parameter("rpm_control_mode").value
        self.hard_stop_mode = self.get_parameter("hard_stop_mode").value

    def parameters_callback(self, params):
        """Callback for parameter changes - enables hot reloading of parameters"""

        for param in params:
            param_name = param.name

            if param_name == "base_width":
                self.base_width = param.value
                self.get_logger().info(f"Updated base_width to {self.base_width}")
            elif param_name == "wheel_radius":
                self.wheel_radius = param.value
                self.get_logger().info(f"Updated wheel_radius to {self.wheel_radius}")
            elif param_name == "gear_ratio":
                self.gear_ratio = param.value
                self.get_logger().info(f"Updated gear_ratio to {self.gear_ratio}")
            elif param_name == "idle_timeout":
                self.idle_timeout = param.value
                self.get_logger().info(f"Updated idle_timeout to {self.idle_timeout}")
            elif param_name == "debug":
                self.debug = param.value
                self.get_logger().info(f"Updated debug to {self.debug}")
            elif param_name == "rpm_control_mode":
                self.rpm_control_mode = param.value
                self.get_logger().info(f"Updated rpm_control_mode to {self.rpm_control_mode}")
            elif param_name == "hard_stop_mode":
                self.hard_stop_mode = param.value
                self.get_logger().info(f"Updated hard_stop_mode to {self.hard_stop_mode}")
            elif param_name == "command_rate":
                self.command_rate = param.value
                self.cmd_timer.cancel()
                self.idle_timer.cancel()
                self.cmd_timer = self.create_timer(1 / self.command_rate, self.send_commands)
                self.idle_timer = self.create_timer(1 / self.command_rate, self.check_idle)
                self.get_logger().info(f"Updated command_rate to {self.command_rate}")
            elif param_name == "status_rate":
                self.status_rate = param.value
                self.status_timer.cancel()
                self.status_timer = self.create_timer(1 / self.status_rate, self.publish_status)
                self.get_logger().info(f"Updated status_rate to {self.status_rate}")
            elif param_name == "odom_rate":
                self.odom_rate = param.value
                self.odom_timer.cancel()
                self.odom_timer = self.create_timer(1 / self.odom_rate, self.publish_odometry)
                self.get_logger().info(f"Updated odom_rate to {self.odom_rate}")
            elif param_name in ["interface", "channel", "bitrate", "database_file"]:
                self.get_logger().warn(
                    f"Parameter '{param_name}' cannot be changed at runtime. Restart the node to apply changes."
                )
                return SetParametersResult(successful=False, reason=f"Parameter '{param_name}' requires node restart")

        return SetParametersResult(successful=True)

    def init_subscribers(self):
        self.twist_sub = self.create_subscription(Twist, "/motors/cmd_vel", self.twist_callback, 10)
        self.deadman_sub = self.create_subscription(Bool, "/teleop_switch", self.brake_callback, 10)

    def init_publishers(self):
        self.left_status_pub = self.create_publisher(SmartecStatus, "/motors/left/status", 10)
        self.right_status_pub = self.create_publisher(SmartecStatus, "/motors/right/status", 10)
        self.odometry_pub = self.create_publisher(Odometry, "/motors/odom", 10)

    def init_timers(self):
        self.cmd_timer = self.create_timer(1 / self.command_rate, self.send_commands)
        self.idle_timer = self.create_timer(1 / self.command_rate, self.check_idle)
        self.status_timer = self.create_timer(1 / self.status_rate, self.publish_status)
        self.odom_timer = self.create_timer(1 / self.odom_rate, self.publish_odometry)

    def init_canbus(self):
        self.can_bus = can.interface.Bus(self.channel, interface=self.interface, bitrate=self.bitrate)
        self.database = cantools.database.load_file(self.database_file)
        self.accepted_ids = [msg.frame_id for msg in self.database.messages]
        self.notifier = can.Notifier(self.can_bus, [self])

    def twist_callback(self, msg):
        # Convert linear and angular velocities to left and right wheel velocities
        self.last_cmd_time = self.get_clock().now()
        left_cmd_rad = (msg.linear.x - msg.angular.z * self.base_width / 2) / self.wheel_radius * self.gear_ratio
        right_cmd_rad = (msg.linear.x + msg.angular.z * self.base_width / 2) / self.wheel_radius * self.gear_ratio
        self.left_command = int(left_cmd_rad / (2 * math.pi) * 60)  # Convert rad/s to RPM
        self.right_command = int(right_cmd_rad / (2 * math.pi) * 60)  # Convert rad/s to RPM

    def brake_callback(self, msg):

        self.teleop_switch = msg.data

    def on_message_received(self, msg):
        # Handle received CAN messages
        if msg.arbitration_id in self.accepted_ids:
            msg_name = self.database.get_message_by_frame_id(msg.arbitration_id).name
            # self.get_logger().info("ALLO1")
            if msg_name == "Left_GDM_General_Status":
                decoded_message = self.database.decode_message(msg.arbitration_id, msg.data)
                self.update_status(decoded_message, self.left_status)
            elif msg_name == "Right_GDM_General_Status":
                decoded_message = self.database.decode_message(msg.arbitration_id, msg.data)
                self.update_status(decoded_message, self.right_status)

    def send_commands(self):
        # Send velocity commands to the motors
        message = self.database.get_message_by_name("Control_GDM_Left_Right")
        command = message.encode(
            {
                "Left_ControlMode": (
                    self.rpm_control_mode
                    if self.teleop_switch == True or self.left_command != 0
                    else self.hard_stop_mode
                ),
                "Left_Command": self.left_command,
                "Right_ControlMode": (
                    self.rpm_control_mode
                    if self.teleop_switch == True or self.right_command != 0
                    else self.hard_stop_mode
                ),
                "Right_Command": self.right_command,
            }
        )
        try:
            self.can_bus.send(can.Message(arbitration_id=message.frame_id, data=command))
        except can.CanOperationError as e:
            self.get_logger().error(f"Failed to transmit: {e}. Retrying...")

    def publish_status(self):
        self.left_status_pub.publish(self.left_status)
        self.right_status_pub.publish(self.right_status)

        # logging the fault codes
        if self.left_status.fault_code != 0 or self.left_status.fault_code != 0:

            self.get_logger().error(
                f"There is a can bus error left : {self.left_status.fault_code}, right: {self.right_status.fault_code}"
            )

    def update_status(self, decoded_message, status):
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = "base_link"
        status.mode = int(decoded_message["ControlMode"])
        status.fault_code = int(decoded_message["LatchedFaultCode"])
        status.speed = float(decoded_message["MeasuredRPM"] * 2 * math.pi / 60.0 / self.gear_ratio)
        status.voltage = float(decoded_message["BusVoltage"])
        status.current = float(decoded_message["AlignedTorqueCurrent"])
        status.power = float(decoded_message["EstimatedPower"])

    def publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_time).nanoseconds * 1e-9  # Convert ns to seconds
        self.last_odom_time = current_time

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Calculate linear and angular velocities
        linear_velocity_x = self.wheel_radius * (self.right_status.speed + self.left_status.speed) / 2.0
        angular_velocity_z = self.wheel_radius * (self.right_status.speed - self.left_status.speed) / self.base_width
        odom_msg.twist.twist.linear.x = linear_velocity_x
        odom_msg.twist.twist.angular.z = angular_velocity_z

        # Update the robot's pose
        self.pose[0] += odom_msg.twist.twist.linear.x * dt * math.cos(self.pose[2])
        self.pose[1] += odom_msg.twist.twist.linear.x * dt * math.sin(self.pose[2])
        self.pose[2] += odom_msg.twist.twist.angular.z * dt

        # Populate the Pose message
        odom_msg.pose.pose.position.x = self.pose[0]
        odom_msg.pose.pose.position.y = self.pose[1]
        quaternion = self.euler_to_quaternion(0.0, 0.0, self.pose[2])
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        self.odometry_pub.publish(odom_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to a quaternion.
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(
            pitch / 2
        ) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(
            pitch / 2
        ) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(
            pitch / 2
        ) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(
            pitch / 2
        ) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def check_idle(self):
        elapsed_time = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed_time > self.idle_timeout and (self.left_command != 0 or self.right_command != 0):
            self.get_logger().info("IDLE")
            self.stop_motors()

    def stop_motors(self):
        self.get_logger().info("STOP")

        self.left_command = 0
        self.right_command = 0
        self.send_commands()

    def shutdown(self):
        self.stop_motors()
        self.notifier.stop()
        self.can_bus.shutdown()


def main(args=None):
    rclpy.init(args=args)
    smartec_driver = SmartecDriver()
    rclpy.spin(smartec_driver)
    smartec_driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
