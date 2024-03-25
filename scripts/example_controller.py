#!/usr/bin/env python3
import rclpy
from discower_transportation.body_controller import BodyControllerNode

def run():
    rclpy.init()
    node = BodyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Stop the controller")
    finally:
        node.get_logger().info("Node is shutting down.")
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run()
