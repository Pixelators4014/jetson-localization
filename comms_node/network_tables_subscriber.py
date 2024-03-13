import rclpy
from rclpy.node import Node

from networktables import NetworkTables

from geometry_msgs.msg import PoseStamped


def serialize_response(poses):
    s = ""
    for pose in poses:
        p = pose.pose
        s += "{} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}\n".format(
            pose.header.stamp.sec, p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    return s


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/visual_slam/tracking/slam_path',
            self.listener_callback,
            10)
        NetworkTables.initialize(server='roborio-XXX-frc.local')
        self.sd = NetworkTables.getTable('Orin')

    def listener_callback(self, msg):
        self.sd.putString("POSE", serialize_response(msg.data))


def main(args=None):
    rclpy.init(args=args)

    subscriber = MinimalSubscriber()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
