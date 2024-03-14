import rclpy

from comms_node.save_map_action import SaveMapActionClient


class SaveMapNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('save_map_node')
        self.declare_parameter('save_location', '/tmp/map.vlm')
        self.save_location = self.get_parameter('save_location').get_parameter_value().string_value
        save = SaveMapActionClient()

        rclpy.spin(save)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        save.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SaveMapNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
