import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from isaac_ros_visual_slam.isaac_ros_visual_slam_interfaces.action import SaveMap


class SaveMapActionClient(Node):

    def __init__(self):
        super().__init__('save_map_action_client')
        self.action_client = ActionClient(self, SaveMap, 'save_map')

    def send_goal(self, map_url):
        goal_msg = SaveMap.Goal()
        goal_msg.map_url = map_url

        self.action_client.wait_for_server()

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)

        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Save Map Action Succeeded')
        else:
            self.get_logger().info('Save Map Action Failed: {}'.format(result.error))
        rclpy.shutdown()

# action_client = FibonacciActionClient()
# action_client.send_goal("/maps/m1.vlm")
# rclpy.spin(action_client)
