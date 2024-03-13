import time

import rclpy
from networktables import NetworkTables
from comms_node.pose_service_client import PoseServiceClient


def serialize_response(poses):
    s = ""
    for pose in poses:
        p = pose.pose
        s += "{} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}\n".format(
            pose.header.stamp.sec, p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    return s


def main(args=None):
    rclpy.init(args=args)
    NetworkTables.initialize(server='roborio-XXX-frc.local')  # TODO: Replace with actual roborio hostname
    sd = NetworkTables.getTable('Orin')
    pose_service_client = PoseServiceClient()
    while sd.getBoolean("RUNNING", False):
        time.sleep(0.1)
    while sd.getBoolean("RUNNING", True):
        response = pose_service_client.send_request(4)
        sd.putString("POSE", serialize_response(response.poses))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
