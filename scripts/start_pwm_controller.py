#!/usr/bin/env python3

# import ROS2 package
import rclpy
from discower_transportation.pwm_controller import PWMNode

def run(ulim):
    rclpy.init()
    node = PWMNode(ulim)
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
    ulim = 1.4
    run(ulim)
