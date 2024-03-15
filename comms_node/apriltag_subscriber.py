import rclpy
from rclpy.node import Node

from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

from networktables import NetworkTables

from comms_node.util import serialize_response


class AprilTagsSubscriber(Node):
    def __init__(self):
        super().__init__('detect_subscriber')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.listener_callback,
            10)
        self.sd = NetworkTables.getTable('Orin')

    def listener_callback(self, msg):
        self.get_logger().info('%s detections' % len(msg.detections))
        self.sd.putString("APRILTAGS", serialize_response(msg.detections))


def main(args=None):
    rclpy.init(args=args)
    subscriber = AprilTagsSubscriber()
    rclpy.spin(subscriber)


if __name__ == '__main__':
    main()
