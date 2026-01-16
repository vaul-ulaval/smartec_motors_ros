#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import can
import cantools
import json


class SmartecListener(Node, can.Listener):

    def __init__(self):
        super().__init__("smartec_frame_publisher")

        # Store latest decoded CAN frame
        self.latest_frame = None

        self.setup_parameters()
        self.init_publishers()
        self.init_canbus()

        # Publish decoded frames at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_frames)

    # ---------------- ROS PARAMETERS ----------------
    def setup_parameters(self):
        self.interface = self.declare_parameter("interface", "socketcan").value

        self.channel = self.declare_parameter("channel", "can0").value

        self.bitrate = self.declare_parameter("bitrate", 500000).value

        self.database_file = self.declare_parameter("database_file", "").value

    # ---------------- ROS PUBLISHERS ----------------
    def init_publishers(self):
        self.frame_publisher = self.create_publisher(String, "/motors/canframes", 10)

    # ---------------- CAN SETUP ----------------
    def init_canbus(self):
        self.can_bus = can.interface.Bus(self.channel, interface=self.interface, bitrate=self.bitrate)

        self.database = cantools.database.load_file(self.database_file)

        self.notifier = can.Notifier(self.can_bus, [self])

    # ================= CAN RX THREAD =================
    def on_message_received(self, msg):
        try:
            message = self.database.get_message_by_frame_id(msg.arbitration_id)
        except KeyError:
            # Unknown CAN ID → ignore
            return

        decoded = message.decode(msg.data)

        self.latest_frame = {"name": message.name, "id": hex(msg.arbitration_id), "signals": decoded}

    # ================= ROS TIMER =================
    def publish_frames(self):
        if self.latest_frame is None:
            return

        ros_msg = String()
        ros_msg.data = json.dumps(self.latest_frame)

        self.frame_publisher.publish(ros_msg)

    # ================= CLEAN SHUTDOWN =================
    def destroy_node(self):
        if hasattr(self, "notifier"):
            self.notifier.stop()

        if hasattr(self, "can_bus"):
            self.can_bus.shutdown()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SmartecListener()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
