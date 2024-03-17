import rclpy

from vision_msgs.msg import Detection2DArray

from networktables import NetworkTables

from comms_node.network_tables_node import NetworkTablesNode
from comms_node.util import serialize_response


class DetectSubscriber(NetworkTablesNode):
    def __init__(self):
        super().__init__('detect_subscriber')
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detections_output',
            self.listener_callback,
            10)
        self.sd = NetworkTables.getTable('Orin')

    def listener_callback(self, msg):
        self.get_logger().info('%s detections' % len(msg.detections))
        self.sd.putString("DETECTIONS", serialize_response(msg.detections))


def main(args=None):
    rclpy.init(args=args)
    subscriber = DetectSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
