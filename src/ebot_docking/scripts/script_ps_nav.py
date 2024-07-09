#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
#from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import time

# Define a class for your ROS2 node
class MyRobotNavigationController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_navigation_controller')
        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()
        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.navigation_loop)
        self.dock_bool_pub = self.create_publisher(Bool,'dock_bool', 10)
        self.undock_bool_pub = self.create_publisher(Bool,'undock_bool', 10)
        self.rack_docked_sub = self.create_subscription(Bool, 'rack_docked_bool',self.rack_docked_callback , 10)
        self.navigator = BasicNavigator()

        self.rack_docked = False
        self.pre_dock_reached = False
        self.arm_pose_reached = False
        self.dock_bool_data = False
        self.undock_bool_data = False


    def rack_docked_callback(self , msg):

        self.rack_docked = msg.data
        

    def navigation_loop(self):


        dock_bool = Bool()
        undock_bool = Bool()
        if (not self.pre_dock_reached):

            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = 0.0
            initial_pose.pose.position.y = 0.0
            initial_pose.pose.orientation.z = 0.0
            initial_pose.pose.orientation.w = 0.0
            self.navigator.setInitialPose(initial_pose)

            self.navigator.waitUntilNav2Active()

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = 0.8
            goal_pose.pose.position.y = 4.5
            goal_pose.pose.orientation.z = 1.00
            goal_pose.pose.orientation.w = 0.00

            self.navigator.goToPose(goal_pose)

            i = 0
            while not self.navigator.isTaskComplete():
                i = i + 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated time of arrival: ' + '{0:.0f}'.format(
                          Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                          + ' seconds.')
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        self.navigator.cancelTask()

            
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.dock_bool_data = True
                self.pre_dock_reached = True
                print('Goal succeeded!')

            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
                self.dock_bool_data = False
            elif result == TaskResult.FAILED:
                print('Goal failed!')
                self.dock_bool_data = False
            else:
                print(result)
                self.dock_bool_data = False

            
        #while not self.docking :
        dock_bool.data = self.dock_bool_data
        self.dock_bool_pub.publish(dock_bool)
        self.get_logger().info(f'Publishing: DOCK = {dock_bool.data}')

        

        if self.rack_docked and (not self.arm_pose_reached):

            goal_pose2 = PoseStamped()
            goal_pose2.header.frame_id = 'map'
            goal_pose2.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose2.pose.position.x = 0.5
            goal_pose2.pose.position.y = -2.40
            goal_pose2.pose.orientation.z = 1.0
            goal_pose2.pose.orientation.w = 0.0

            self.navigator.goToPose(goal_pose2)
      
                
            i = 0
            while not self.navigator.isTaskComplete():
                i = i + 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated time of arrival: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                        + ' seconds.')
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        self.navigator.cancelTask()

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.arm_pose_reached = True
                self.undock_bool_data = True
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                self.undock_bool_data = False
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                self.undock_bool_data = False
                print('Goal failed!')
            else:
                self.undock_bool_data = False
                print(result)
#
        if self.arm_pose_reached:
            self.navigator.lifecycleShutdown()
#
        undock_bool.data = self.undock_bool_data
        self.undock_bool_pub.publish(undock_bool)
        self.get_logger().info(f'Publishing: UNDOCK = {undock_bool.data}')

   
# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)
    my_robot_navigation_controller = MyRobotNavigationController()
    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_navigation_controller)
    executor.spin()
    my_robot_navigation_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
