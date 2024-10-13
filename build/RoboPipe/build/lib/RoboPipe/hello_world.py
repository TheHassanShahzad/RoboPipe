#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args) #passes all arguments of main into rclpy
    node = Node("py_test") #name of node is py_test within code
    node.get_logger().info("Hello ROS2") #prints hello ROS2
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()