import rclpy

from pose_service_client import PoseServiceClient


def main(args=None):
    rclpy.init(args=args)

    client = PoseServiceClient()
    while True:
        response = client.send_request(4)
        client.get_logger().info("vslam_pose: " + response.poses)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
