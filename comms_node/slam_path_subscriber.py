import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path


class SlamPathSubscriber(Node):

    def __init__(self, name):
        super().__init__(name)
        self.subscription = self.create_subscription(
            Path,
            '/visual_slam/tracking/slam_path',
            self.listener_callback,
            10)
