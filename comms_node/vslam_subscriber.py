import rclpy
from rclpy.node import Node

from networktables import NetworkTables
from nav_msgs.msg import Path

from comms_node.util import serialize_response


class NetworkTablesSlamPathSubscriber(Node):
    def __init__(self):
        super().__init__('network_tables_pose_subscriber')
        self.subscription = self.create_subscription(
            Path,
            '/visual_slam/tracking/slam_path',
            self.listener_callback,
            10)
        self.sd = NetworkTables.getTable('Orin')

    def listener_callback(self, msg):
        self.get_logger().info('Received SLAM path')
        self.sd.putString("POSE", serialize_response(msg.poses))


def main(args=None):
    rclpy.init(args=args)

    subscriber = NetworkTablesSlamPathSubscriber()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
