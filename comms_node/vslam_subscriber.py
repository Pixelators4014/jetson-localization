import rclpy

from networktables import NetworkTables
from nav_msgs.msg import Path

from comms_node.network_tables_node import NetworkTablesNode
from comms_node.util import serialize_response


class NetworkTablesSlamPathSubscriber(NetworkTablesNode):
    def __init__(self):
        super().__init__('vslam_pose_subscriber')
        self.subscription = self.create_subscription(
            Path,
            '/visual_slam/tracking/slam_path',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('Received SLAM path')
        self.sd.putString("POSE", serialize_response(msg.poses))


def main(args=None):
    rclpy.init(args=args)
    subscriber = NetworkTablesSlamPathSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
