import rclpy

from comms_node.slam_path_subscriber import SlamPathSubscriber


class LogSlamPathSubscriber(SlamPathSubscriber):

    def __init__(self):
        super().__init__('log_pose_subscriber')

    def listener_callback(self, msg):
        if len(msg.poses) > 0:
            self.get_logger().info('I heard: "%s"' % msg.poses[-1])


def main(args=None):
    rclpy.init(args=args)

    subscriber = LogSlamPathSubscriber()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
