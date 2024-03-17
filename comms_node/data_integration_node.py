import json

import rclpy

from comms_node.network_tables_node import NetworkTablesNode


class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


class DataIntegrationNode(NetworkTablesNode):
    def __init__(self):
        super().__init__('data_integration_node')
        self.starting_pose = Pose(**json.loads(self.sd.getString("POSE")))
        self.loop()

    def loop(self):
        while rclpy.ok():
            vslam_path = self.sd.getString("VSLAM_PATH")
            if vslam_path is not None:
                if len(vslam_path) > 0:
                    last_vslam_path = vslam_path[-1]
                    theta = last_vslam_path[""]
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = DataIntegrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
