#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import can
import math


class SmartecListener(Node, can.Listener):

    def __init__(self):
        super().__init__('smartec_frame_publisher')

        self.last_cmd_time = self.get_clock().now()
        self.last_odom_time = self.get_clock().now()
        self.frames = String()

        self.setup_parameters()
        self.init_publishers()
        self.init_canbus()

        self.stop_motors()

    def setup_parameters(self):
        self.interface = self.declare_parameter('interface', 'socketcan').value
        self.channel = self.declare_parameter('channel', 'can32').value
        self.bitrate = self.declare_parameter('bitrate', 500000).value
        self.database_file = self.declare_parameter('database_file', '').value

    def init_publishers(self):
        self.frame_publisher = self.create_publisher(String, '/motors/canframes', 10)

    def init_canbus(self):
        self.can_bus = can.interface.Bus(self.channel, interface=self.interface, bitrate=self.bitrate)
        self.database = cantools.database.load_file(self.database_file)
        self.accepted_ids = [msg.frame_id for msg in self.database.messages]
        self.notifier = can.Notifier(self.can_bus, [self])

    def on_message_received(self, msg):
        # Handle received CAN messages
        if msg.arbitration_id in self.accepted_ids:
            # self.get_logger().info("ALLO1")
            decoded_message = self.database.decode_message(msg.arbitration_id, msg.data)

            self.frames= decoded_message

    def publish_frames(self): 
        self.left_status_pub.publish(self.left_status)
        self.frame_publisher.publisher(self.frames)

    def shutdown(self):
        self.notifier.stop()

def main(args=None):
    rclpy.init(args=args)
    smartec_driver = SmartecListener()
    rclpy.spin(smartec_driver)
    smartec_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
