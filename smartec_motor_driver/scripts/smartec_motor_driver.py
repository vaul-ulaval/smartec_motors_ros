#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float32, Bool
from smartec_msgs.msg import SmartecStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import can
import cantools
import math

class SmartecDriver(Node, can.Listener):

    def __init__(self):
        super().__init__('smartec_motor_driver')

        self.last_cmd_time = self.get_clock().now()
        self.last_odom_time = self.get_clock().now()
        self.left_status = SmartecStatus()
        self.right_status = SmartecStatus()
        self.pose = [0, 0, 0]
        self.teleop_switch = False


        self.setup_parameters()
        self.init_subscribers()
        self.init_publishers()
        self.init_timers()    
        self.init_canbus()

        self.stop_motors()

    def setup_parameters(self):
        self.interface = self.declare_parameter('interface', 'socketcan').value
        self.channel = self.declare_parameter('channel', 'can32').value
        self.bitrate = self.declare_parameter('bitrate', 500000).value
        self.database_file = self.declare_parameter('database_file', '').value
        self.command_rate = self.declare_parameter('command_rate', 50).value
        self.status_rate = self.declare_parameter('status_rate', 20).value
        self.odom_rate = self.declare_parameter('odom_rate', 50).value
        self.base_width = self.declare_parameter('base_width', 1.0).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.2).value
        self.gear_ratio = self.declare_parameter('gear_ratio', 17.37).value
        self.idle_timeout = self.declare_parameter('idle_timeout', 0.5).value
        self.debug = self.declare_parameter('debug', False).value
        # These modes match the names give by HydroGear
        self.rpm_control_mode = self.declare_parameter('rpm_control_mode', 2).value
        self.hard_stop_mode = self.declare_parameter('hard_stop_mode', 1).value
        

    def init_subscribers(self):
        self.twist_sub = self.create_subscription(Twist, '/motors/cmd_vel', self.twist_callback, 10)
        self.deadman_sub = self.create_subscription(Bool, '/teleop_switch', self.brake_callback, 10)

    def init_publishers(self):
        self.left_status_pub = self.create_publisher(SmartecStatus, '/motors/left/status', 10)
        self.right_status_pub = self.create_publisher(SmartecStatus, '/motors/right/status', 10)
        self.odometry_pub = self.create_publisher(Odometry, '/motors/odom', 10)

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
            if msg_name == 'Left_GDM_General_Status':
                decoded_message = self.database.decode_message(msg.arbitration_id, msg.data)
                self.update_status(decoded_message, self.left_status)
            elif msg_name == 'Right_GDM_General_Status':
                decoded_message = self.database.decode_message(msg.arbitration_id, msg.data)
                self.update_status(decoded_message, self.right_status)

    def send_commands(self):
        # Send velocity commands to the motors
        message = self.database.get_message_by_name('Control_GDM_Left_Right')
        command = message.encode({
            'Left_ControlMode': self.rpm_control_mode if self.teleop_switch == True or self.left_command != 0 else self.hard_stop_mode,
            'Left_Command': self.left_command,
            'Right_ControlMode': self.rpm_control_mode if self.teleop_switch == True or self.right_command != 0 else self.hard_stop_mode,
            'Right_Command': self.right_command
        })
        try:
            self.can_bus.send(can.Message(arbitration_id=message.frame_id, data=command))
        except can.CanOperationError as e:
            self.get_logger().error(f"Failed to transmit: {e}. Retrying...")

    def publish_status(self): 
        self.left_status_pub.publish(self.left_status)
        self.right_status_pub.publish(self.right_status)

    def update_status(self, decoded_message, status):
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = 'base_link'
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
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Calculate linear and angular velocities
        linear_velocity_x = self.wheel_radius * (self.right_status.speed + self.left_status.speed) / 2.0
        angular_velocity_z = self.wheel_radius * (self.right_status.speed - self.left_status.speed) / self.base_width
        odom_msg.twist.twist.linear.x = linear_velocity_x
        odom_msg.twist.twist.angular.z = angular_velocity_z

        # Update the robot's pose
        self.pose[2] += odom_msg.twist.twist.angular.z * dt
        self.pose[0] += odom_msg.twist.twist.linear.x * dt * math.cos(self.pose[2])
        self.pose[1] += odom_msg.twist.twist.linear.x * dt * math.sin(self.pose[2])

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
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def check_idle(self):
        elapsed_time = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9 
        if elapsed_time > self.idle_timeout and (self.left_command != 0 or self.right_command !=0):
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


if __name__ == '__main__':
    main()
