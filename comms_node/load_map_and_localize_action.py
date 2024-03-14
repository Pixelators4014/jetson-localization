import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from isaac_ros_visual_slam.isaac_ros_visual_slam_interfaces.action import LoadMapAndLocalize
from geometry_msgs.msg import Vector3

class LoadMapActionClient(Node):

    def __init__(self):
        super().__init__('load_map_action_client')
        self.action_client = ActionClient(self, LoadMapAndLocalize, 'load_map')

    def send_goal(self, map_url, location_guess):
        """
        Send a goal to the action server to load a map and localize near a point
        :param map_url: The URL of the map to load
        :param location_guess: The 3D point to localize near, use a 3 element list or tuple
        """
        goal_msg = LoadMapAndLocalize.Goal()
        goal_msg.map_url = map_url
        localize_near_point = Vector3()
        localize_near_point.x = location_guess[0]
        localize_near_point.y = location_guess[1]
        localize_near_point.z = location_guess[2]
        goal_msg.localize_near_point = localize_near_point

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
            self.get_logger().info('Load Map Action Succeeded')
        else:
            self.get_logger().info('Load Map Action Failed: {}'.format(result.error))
        rclpy.shutdown()

# action_client = LoadMapActionClient()
# action_client.send_goal("/maps/m1.vlm", )
# rclpy.spin(action_client)
