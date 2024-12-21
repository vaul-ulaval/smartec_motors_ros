#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from smartec_msgs.msg import SmartecStatus
import can
import cantools
import math


class SmartecDriver(Node, can.Listener):

    def __init__(self):
        super().__init__('smartec_motor_driver')

        self.interface = self.declare_parameter('interface', 'socketcan').value
        self.channel = self.declare_parameter('channel', 'can32').value
        self.bitrate = self.declare_parameter('bitrate', 500000).value
        self.database_file = self.declare_parameter('database_file', '').value
        self.debug = self.declare_parameter('debug', False).value
        self.command_rate = self.declare_parameter('command_rate', 20).value
        self.status_rate = self.declare_parameter('status_rate', 50).value

        self.left_vel_sub = self.create_subscription(Float32, '/motors/left/velocity', self.vel_left_callback, 10)
        self.right_vel_sub = self.create_subscription(Float32, '/motors/right/velocity', self.vel_right_callback, 10)
        self.left_velocity = 0.0
        self.right_velocity = 0.0

        self.left_status_pub = self.create_publisher(SmartecStatus, '/motors/left/status', 10)
        self.right_status_pub = self.create_publisher(SmartecStatus, '/motors/right/status', 10)
        self.left_status = SmartecStatus()
        self.right_status = SmartecStatus()

        self.can_bus = can.interface.Bus(self.channel, interface=self.interface, bitrate=self.bitrate)
        self.database = cantools.database.load_file(self.database_file)
        self.accepted_ids = [msg.frame_id for msg in self.database.messages]

        self.notifier = can.Notifier(self.can_bus, [self])   # Calls on_message_received when a message is received
        self.cmd_timer = self.create_timer(1/self.command_rate, self.send_commands)
        self.status_timer = self.create_timer(1/self.status_rate, self.publish_messages)
        
    def vel_left_callback(self, msg):
        self.left_velocity = int(msg.data / (2 * math.pi) * 60)  # Convert rad/s to RPM

    def vel_right_callback(self, msg):
        self.right_velocity = int(msg.data / (2 * math.pi) * 60) # Convert rad/s to RPM

    def on_message_received(self, msg):
        # Handle received CAN messages
        if msg.arbitration_id in self.accepted_ids:
            decoded_message = self.database.decode_message(msg.arbitration_id, msg.data)
            if decoded_message.name == 'Left_GDM_General_Status':
                self.update_status(decoded_message, self.left_status)
            elif decoded_message.name == 'Right_GDM_General_Status':
                self.update_status(decoded_message, self.right_status)

    def send_commands(self):
        # Send velocity commands to the motors
        command = self.database.encode_message('Control_GDM_Left_Right', {
            'Left_ControlMode': 2,
            'Left_Command': self.left_velocity,
            'Right_ControlMode': 2,
            'Right_Command': self.right_velocity
        })
        self.can_bus.send(command)

    def publish_messages(self): 
        self.left_status_pub.publish(self.left_status)
        self.right_status_pub.publish(self.right_status)

    def update_status(self, decoded_message, status):
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = 'base_link'
        status.mode = decoded_message["ControlMode"]
        status.fault_code = decoded_message["LatchedFaultCode"]
        status.speed = decoded_message["MeasuredRPM"] * 2.0 * math.pi / 60.0
        status.voltage = decoded_message["BusVoltage"]
        status.current = decoded_message["AlignedTorqueCurrent"]
        status.power = decoded_message["EstimatedPower"]

    def stop_motors(self):
        self.left_velocity = 0
        self.right_velocity = 0
        self.send_commands()

    def shutdown(self):
        self.stop_motors()
        self.notifier.stop()
        self.can_bus.shutdown()
        super().shutdown()


def main(args=None):
    rclpy.init(args=args)
    smartec_driver = SmartecDriver()
    rclpy.spin(smartec_driver)
    smartec_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()