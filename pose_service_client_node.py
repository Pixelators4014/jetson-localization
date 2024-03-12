import sys

from isaac_ros_visual_slam.isaac_ros_visual_slam_interfaces.srv import GetAllPoses
import rclpy
from rclpy.node import Node
from networktables import NetworkTables


class PoseServiceClient(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(GetAllPoses, 'get_all_poses')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetAllPoses.Request()
        NetworkTables.initialize(server='roborio-XXX-frc.local')
        self.sd = NetworkTables.getTable('SmartDashboard')

        # self.sd.putNumber('someNumber', 1234)
        # otherNumber = self.sd.getNumber('otherNumber')

    def send_request(self, a, b):
        self.req.max_count = 4
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def serialize_response(poses):
    return ""

def main(args=None):
    rclpy.init(args=args)

    client = PoseServiceClient()
    while True:
        response = client.send_request(int(sys.argv[1]), int(sys.argv[2]))
        client.sd.putBoolean("vslam_success", response.success)
        client.sd.putString("vslam_pose", serialize_response(response.poses))
        if not client.sd.getBoolean("vslam_running"):
            break
    rclpy.shutdown()


if __name__ == '__main__':
    main()
