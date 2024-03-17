from threading import Thread

import rclpy

from comms_node.apriltag_subscriber import AprilTagsSubscriber
from comms_node.data_integration_node import DataIntegrationNode
from comms_node.detect_subscriber import DetectSubscriber
from comms_node.vslam_subscriber import VSlamPathSubscriber


def main(args=None):
    rclpy.init(args=args)

    april_tags_subscriber = AprilTagsSubscriber()
    data_integration_subscriber = DataIntegrationNode()
    detect_subscriber = DetectSubscriber()
    vslam_subscriber = VSlamPathSubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()

    executor.add_node(april_tags_subscriber)
    executor.add_node(data_integration_subscriber)
    executor.add_node(detect_subscriber)
    executor.add_node(vslam_subscriber)

    et = Thread(target=executor.spin)
    et.start()
    et.join()

    april_tags_subscriber.destroy_node()
    data_integration_subscriber.destroy_node()
    detect_subscriber.destroy_node()
    vslam_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
