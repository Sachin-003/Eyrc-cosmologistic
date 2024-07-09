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
from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
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
        self.cli = self.create_client(DockSw, 'dock_control' , callback_group=self.callback_group)
        self.pub1 = self.create_publisher(Bool , 'first_docked', 10 )
        self.pub2 = self.create_publisher(Bool , 'second_docked', 10 )
        self.req = DockSw.Request()
        self.navigator = BasicNavigator()
        self.rack1_placed = False
        self.rack2_placed = False
        

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
        goal_pose.pose.position.x = 0.8
        goal_pose.pose.position.y = 4.5
        goal_pose.pose.orientation.z = 1.0
        goal_pose.pose.orientation.w = 0.0

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
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print(result)

        self.req.linear_dock = True
        self.req.orientation_dock = True
        self.req.distance = 0.00
        self.req.orientation = 1.57
        self.rack_no = 3
        self.cool = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.cool)
        #time.sleep(10)
        self.result = self.cool.result().success
        print(self.result)


        
        
        
        if self.result :

            


            

            #goal_pose2 = PoseStamped()
            #goal_pose2.header.frame_id = 'map'
            #goal_pose2.header.stamp = self.navigator.get_clock().now().to_msg()
            #goal_pose2.pose.position.x = 0.5
            #goal_pose2.pose.position.y = -2.44
            #goal_pose2.pose.orientation.z = -0.71
            #goal_pose2.pose.orientation.w = 0.71
#
            #self.navigator.goToPose(goal_pose2)
      #
            #    
            #i = 0
            #while not self.navigator.isTaskComplete():
            #    i = i + 1
            #    feedback = self.navigator.getFeedback()
            #    if feedback and i % 5 == 0:
            #        print('Estimated time of arrival: ' + '{0:.0f}'.format(
            #            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
            #            + ' seconds.')
            #        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            #            self.navigator.cancelTask()
#
            #result = self.navigator.getResult()
            #if result == TaskResult.SUCCEEDED:
            #    print('Goal succeeded!')
            #elif result == TaskResult.CANCELED:
            #    print('Goal was canceled!')
            #elif result == TaskResult.FAILED:
            #    print('Goal failed!')
            #else:
            #    print(result)

            goal_pose4 = PoseStamped()
            goal_pose4.header.frame_id = 'map'
            goal_pose4.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose4.pose.position.x = 0.5
            goal_pose4.pose.position.y = -2.44
            goal_pose4.pose.orientation.z = -1.0
            goal_pose4.pose.orientation.w = 0.0

            self.navigator.goToPose(goal_pose4)
      
                
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


            

            self.req.linear_dock = False
            self.req.orientation_dock = False
            self.req.distance = 0.00
            self.req.orientation = 1.57
            self.rack_no = 3
            self.cool2 = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.cool2)
            self.result2 = self.cool2.result().success
            print(f"success of undocking = {self.result2}")

            if self.result2:
                self.rack1_placed = True
                first_docked = Bool()
                first_docked.data = self.rack1_placed
                self.pub1.publish(first_docked)
                self.get_logger().info(f'publishing:{first_docked.data}')
                goal_pose3 = PoseStamped()
                goal_pose3.header.frame_id = 'map'
                goal_pose3.header.stamp = self.navigator.get_clock().now().to_msg()
                goal_pose3.pose.position.x = 2.3
                goal_pose3.pose.position.y = 2.5
                goal_pose3.pose.orientation.z = -0.71
                goal_pose3.pose.orientation.w = 0.71

                self.navigator.goToPose(goal_pose3)


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

                self.req.linear_dock = True
                self.req.orientation_dock = True
                self.req.distance = 0.00
                self.req.orientation = 1.57
                self.rack_no = 3
                self.k = self.cli.call_async(self.req)
                rclpy.spin_until_future_complete(self, self.k)
                #time.sleep(10)
                self.result3 = self.k.result().success
                print(self.result3)
                if self.result3:

                    goal_pose3 = PoseStamped()
                    goal_pose3.header.frame_id = 'map'
                    goal_pose3.header.stamp = self.navigator.get_clock().now().to_msg()
                    goal_pose3.pose.position.x = 1.35
                    goal_pose3.pose.position.y = -3.155
                    #goal_pose3.pose.orientation.z = -0.71
                    #goal_pose3.pose.orientation.w = 0.71
                    goal_pose3.pose.orientation.z = 0.0
                    goal_pose3.pose.orientation.w = 1.0

                    self.navigator.goToPose(goal_pose3)


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



                    self.req.linear_dock = False
                    self.req.orientation_dock = False
                    self.req.distance = 0.00
                    self.req.orientation = 1.57
                    self.rack_no = 3
                    self.k2 = self.cli.call_async(self.req)
                    rclpy.spin_until_future_complete(self, self.k2)
                    self.result4 = self.k2.result().success
                    print(f"success of undocking = {self.result4}")
                    if self.result4:
                        self.rack2_placed = True

                        second_docked = Bool()
        
                        second_docked.data = self.rack2_placed

                        self.pub2.publish(second_docked)
                        self.get_logger().info(f'publishing : {second_docked.data}')

        




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
