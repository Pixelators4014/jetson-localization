from rclpy.node import Node
import ntcore


class NetworkTablesNode(Node):
    def __init__(self):
        super().__init__('network_tables_server')

    def start(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startServer()

    def stop(self):
        self.inst.stopServer()


def main(args=None):
    rclpy.init(args=args)
    server = NetworkTablesNode()
    server.start()
    rclpy.spin(server)
    server.destroy_node()
    server.stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
