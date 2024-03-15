import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray


class DetectSubscriber(Node):
    def __init__(self):
        super().__init__('detect_subscriber')
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detections_output',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info("test")
        for detection in msg.detections:
            self.get_logger().info(
                f'Detected: {detection.bbox.center.x:.2f} {detection.bbox.center.y:.2f} {detection.bbox.size_x:.2f} {detection.bbox.size_y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    subscriber = DetectSubscriber()
    rclpy.spin(subscriber)


if __name__ == '__main__':
    main()
