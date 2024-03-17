from rclpy.node import Node
from networktables import NetworkTables


class NetworkTablesNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sd = NetworkTables.getTable("Orin")
        self.declare_parameter('network_tables_server', 'roborio-XXX-frc.local')
        NetworkTables.initialize(
            server=self.get_parameter('network_tables_server').get_parameter_value().string_value)
