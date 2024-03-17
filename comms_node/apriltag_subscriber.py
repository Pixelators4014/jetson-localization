import rclpy

from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

from comms_node.network_tables_node import NetworkTablesNode
from comms_node.util import serialize_response


class AprilTagsSubscriber(NetworkTablesNode):
    def __init__(self):
        super().__init__('april_tags_subscriber')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('%s detections' % len(msg.detections))
        self.sd.putString("RAW_APRILTAGS", serialize_response(msg.detections))


def main(args=None):
    rclpy.init(args=args)
    subscriber = AprilTagsSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
