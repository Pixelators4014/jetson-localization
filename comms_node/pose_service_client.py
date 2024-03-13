from isaac_ros_visual_slam_interfaces.srv import GetAllPoses
import rclpy
from rclpy.node import Node

class PoseServiceClient(Node):
    def __init__(self):
        super().__init__('get_all_poses_client_async')
        self.cli = self.create_client(GetAllPoses, '/visual_slam/get_all_poses')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetAllPoses.Request()

        # self.sd.putNumber('someNumber', 1234)
        # otherNumber = self.sd.getNumber('otherNumber')

    def send_request(self, max_count):
        self.req.max_count = max_count
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
