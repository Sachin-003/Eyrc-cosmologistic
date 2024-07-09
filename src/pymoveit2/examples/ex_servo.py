#!/usr/bin/env python3
"""
Example of using MoveIt 2 Servo to perform a circular motion.
`ros2 run pymoveit2 ex_servo.py`
"""


from math import cos, sin
import math, time
from copy import deepcopy
import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_servo")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    __twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    
    def servo_circular_motion():
        __twist_msg = TwistStamped()
        current_time = node.get_clock().now()
        __twist_msg.header.frame_id = ur5.base_link_name()
        __twist_msg.header.stamp = current_time.to_msg()
        __twist_msg.twist.linear.x = 0.0
        __twist_msg.twist.linear.y = 0.2
        __twist_msg.twist.linear.z = 0.0
        __twist_msg.twist.angular.x = 0.0
        __twist_msg.twist.angular.y = 0.0
        __twist_msg.twist.angular.z = 0.0
        __twist_pub.publish(__twist_msg)

    node.create_timer(0.2, servo_circular_motion)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()