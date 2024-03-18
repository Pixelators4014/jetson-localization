from rclpy.node import Node
import ntcore


class NetworkTablesNode(Node):
    def __init__(self, name):
        super().__init__(name)
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startClient4(name)
        inst.setServer("127.00.0.1")
        self.sd = inst.getTable("Orin")
