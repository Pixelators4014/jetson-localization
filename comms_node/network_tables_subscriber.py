import rclpy
from rclpy.node import Node

from networktables import NetworkTables


from comms_node.slam_path_subscriber import SlamPathSubscriber


def serialize_response(poses):
    s = ""
    for pose in poses:
        p = pose.pose
        s += "{} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}\n".format(
            pose.header.stamp.sec, p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    return s


class NetworkTablesSlamPathSubscriber(SlamPathSubscriber):
    def __init__(self):
        super().__init__('network_tables_pose_subscriber')
        self.sd = NetworkTables.getTable('Orin')

    def listener_callback(self, msg):
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
