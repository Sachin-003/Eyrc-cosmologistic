#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types
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
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)

        # Subscribe to ultrasonic sensor data for distance measurements
        #self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        # Add another one here
        #self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        #self.imu_sub =  self.create_subscription(Imu, '/imu', self.imu_callback , 10)
        self.ultra_sub = self.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', self.ultra_callback, 10)
        self.imu_sub =  self.create_subscription(Float32, '/orientation', self.imu_callback , 10)

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)
        self.robot_pose = [0,0,0]
        self.robot_vel_x = 0.0
        self.robot_vel_y = 0.0
        self.robot_rotvel = 0.0
        self.robot_orientation_z = 0.0
        self.robot_orientation_w = 0.0
        # Create a publisher for sending velocity commands to the robot
        self.publisher_cmd_vel = self.create_publisher(Twist,'/cmd_vel',10)  
        self.link_attach_cli = self.create_client(AttachLink, '/ATTACH_LINK')
        self.link_detach_cli = self.create_client(DetachLink, '/DETACH_LINK')
        # Initialize all  flags and parameters here
        self.is_docking = False
        self.dock_aligned = False
        self.rack_docked = False
        self.undocking = False
        self.orientation_docked = False
        self.rack_undocked = False
        self.rack_x = 2.02
        self.rack_y = -8.09
        self.rack_z = 0.71
        self.rack_w = 0.71
        self.usrleft_value = 0.00
        self.usrright_value = 0.00
        self.ap1_x = 0.5
        self.ap1_y = -2.455
        self.ap1_oz = 1.0
        self.ap1_ow = 0.0
        self.navigator = BasicNavigator()
        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)
        
        
    # Callback function for odometry data
    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw
        self.robot_vel_x = msg.twist.twist.linear.x
        self.robot_vel_y = msg.twist.twist.linear.y
        self.robot_rotvel = msg.twist.twist.angular.z
        self.robot_orientation_z = quaternion_array.z
        self.robot_orientation_w = quaternion_array.w
        
    # Callback function for the left ultrasonic sensor
    #def ultrasonic_rl_callback(self, msg):
    #    self.usrleft_value = msg.range
        
    #def ultrasonic_rr_callback(self, msg):
    #    self.usrright_value = msg.range
    def ultra_callback(self,msg):

        self.usrleft_value= msg.data[4]
        self.usrright_value = msg.data[5] 


    def imu_callback(self, msg):
        self.orientation_values = msg.orientation
        
        #quats = [self.orientation_values.x , self.orientation_values.y , self.orientation_values.z , self.orientation_values.w]
        #_, _, yaw = euler_from_quaternion(quats)
        #self.robot_pose[2]=yaw
    
    
        
    # Utility function to normalize angles within the range of -π to π (OPTIONAL)
    #def normalize_angle(self, angle):
        
    #    pass

    # Main control loop for managing docking behavior

    def controller_loop(self):

        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and execution
        self.dist_from_rack = math.sqrt(((self.robot_pose[0]-self.rack_x)**2)+((self.robot_pose[1]-self.rack_y)**2))
        self.dist_from_ap1 = math.sqrt(((self.robot_pose[0]-self.ap1_x)**2)+((self.robot_pose[1]-self.ap1_y)**2))

        v_msg = Twist()

        

        if self.is_docking :

            if ((self.robot_pose[2]<1.57)and(abs(self.robot_pose[2]-1.57)>0.01)and(self.orientation_docked == False)):
                v_msg.linear.x = 0.0
                v_msg.angular.z = 0.2
                self.publisher_cmd_vel.publish(v_msg)
            if ((self.robot_pose[2]>1.57)and(abs(self.robot_pose[2]-1.57)>0.01)and(self.orientation_docked == False)):
                v_msg.linear.x = 0.0
                v_msg.angular.z = -0.2
                self.publisher_cmd_vel.publish(v_msg)
            if ((abs(self.robot_pose[2]-1.57)<0.01)and(self.orientation_docked == False)):
                v_msg.linear.x = 0.0
                v_msg.angular.z = 0.0
                self.publisher_cmd_vel.publish(v_msg)
                self.orientation_docked = True
                print("Angle adjusted")

            if ((abs(self.usrleft_value)<0.1)and(abs(self.usrright_value)<0.1)):
                v_msg.linear.x = 0.0
                v_msg.angular.z = 0.0
                self.publisher_cmd_vel.publish(v_msg)
                #while not self.link_attach_cli.wait_for_service(timeout_sec=1.0):
                #    self.get_logger().info('Link attacher service not available, waiting again...')

                #req = AttachLink.Request()
                #req.model1_name =  'ebot'     
                #req.link1_name  = 'ebot_base_link'       
                #req.model2_name =  'rack3'      
                #req.link2_name  = 'link'  
                #self.link_attach_cli.call_async(req)
                print("rack docked successfully...")
                self.rack_docked = True
                self.dock_aligned = True
                self.is_docking = False
                
                
                
            else:
                if ((abs(self.usrleft_value - self.usrright_value)>0.005)and(self.usrleft_value>self.usrright_value)and(self.orientation_docked == True)):
                    v_msg.linear.x = 0.0
                    v_msg.angular.z = 0.5
                    self.publisher_cmd_vel.publish(v_msg)
                    

                if ((abs(self.usrleft_value - self.usrright_value)>0.005)and(self.usrleft_value<self.usrright_value)and(self.orientation_docked == True)):
                    v_msg.linear.x = 0.0
                    v_msg.angular.z = -0.5
                    self.publisher_cmd_vel.publish(v_msg)
                    

                if ((abs(self.usrleft_value - self.usrright_value)<0.005)and(self.orientation_docked == True)):
                    v_msg.linear.x = -0.15
                    v_msg.angular.z = 0.0
                    self.publisher_cmd_vel.publish(v_msg)

        if self.undocking :

            
            #print(f'yaw = {self.robot_pose[2]}')
            #print(f'required yaw = 3.14')

            if ((self.robot_pose[2]<3.14)and(abs(self.robot_pose[2]-3.14)>0.01)):
                v_msg.linear.x = 0.0
                v_msg.angular.z = 0.2
                self.publisher_cmd_vel.publish(v_msg)
            if ((self.robot_pose[2]>3.14)and(abs(self.robot_pose[2]-3.14)>0.01)):
                v_msg.linear.x = 0.0
                v_msg.angular.z = -0.2
                self.publisher_cmd_vel.publish(v_msg)
            if ((abs(self.robot_pose[2]-3.14)<0.01)):
                v_msg.linear.x = 0.0
                v_msg.angular.z = 0.0
                self.publisher_cmd_vel.publish(v_msg)
                print("Rack Undocked")
                #while not self.link_detach_cli.wait_for_service(timeout_sec=1.0):
                #    self.get_logger().info('Link detacher service not available, waiting again...')

                #req = DetachLink.Request()
                #req.model1_name =  'ebot'     
                #req.link1_name  = 'ebot_base_link'       
                #req.model2_name =  'rack3'       
                #req.link2_name  = 'link'  
                #self.link_detach_cli.call_async(req)
                self.rack_undocked = True
                self.dock_aligned = True
            
            

            


        pass



    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
        
        
        if request.linear_dock and request.orientation_dock :
            self.is_docking = True
            
        if (not request.linear_dock) and (not request.orientation_dock) :
            self.undocking = True
            self.dock_aligned = False
        
        self.get_logger().info("Docking started!")

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())

        # Wait until the robot is aligned for docking
        while not self.dock_aligned:
            self.get_logger().info("Waiting for alignment...")
            rate.sleep()

        # Set the service response indicating success
        if request.linear_dock and request.orientation_dock :
            if self.rack_docked:
                response.success = True
            else:
                response.success = False

        if (not request.linear_dock) and (not request.orientation_dock) :
            if self.rack_undocked:
                response.success = True
            else:
                response.success = False


        
        response.message = "Docking control initiated"
        print(response)
        return response

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)
    my_robot_docking_controller = MyRobotDockingController()
    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)
    executor.spin()
    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
