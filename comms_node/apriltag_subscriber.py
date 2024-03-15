import rclpy
from rclpy.node import Node

from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray


class DetectSubscriber(Node):
    def __init__(self):
        super().__init__('detect_subscriber')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        for detection in msg.detections:
            self.get_logger().info(
                f"{detection.pose.pose.pose.position.x:.2f}, {detection.pose.pose.pose.position.y:.2f}, {detection.pose.pose.pose.position.z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    subscriber = DetectSubscriber()
    rclpy.spin(subscriber)


if __name__ == '__main__':
    main()
