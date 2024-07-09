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
        self.arm_flag_pub = self.create_publisher(Bool,'arm_flag', 10)
        self.navigator = BasicNavigator()
        

    def navigation_loop(self):

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
        goal_pose.pose.position.x = 1.1
        goal_pose.pose.position.y =1.97
        goal_pose.pose.orientation.z = 0.00
        goal_pose.pose.orientation.w = 1.00

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
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print(result)

        

        
        
            
            



        self.navigator.lifecycleShutdown()








        

        
           
        
                
           
        


   
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
