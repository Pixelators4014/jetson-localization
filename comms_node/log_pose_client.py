import rclpy

from comms_node.pose_service_client import PoseServiceClient


def main(args=None):
    rclpy.init(args=args)

    client = PoseServiceClient()
    while True:
        response = client.send_request(1)
        client.get_logger().info("vslam_pose: " + str(response.poses[0]))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
