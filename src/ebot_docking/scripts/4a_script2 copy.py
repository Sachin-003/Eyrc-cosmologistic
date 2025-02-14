'''
*****************************************************************************************
*
*        		===============================================
*           		Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement docking with ultrasonic and imu for the Cosmo Logistic (CL) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*  Filename:			ultra_docking_implementation.py
*  Created:				09/10/2023
*  Last Modified:	    13/10/2023
*  Modified by:         Archit
*  Author:				Archit, e-Yantra Team
*****************************************************************************************
'''

# Ensure that the robot is at the home position before starting the script.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import Float32MultiArray, Float32
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ebot_docking.srv import DockSw
from tf_transformations import euler_from_quaternion
from usb_relay.srv import RelaySw
from std_msgs.msg import Bool
import math


class ThemeImplementation(Node):
    def __init__(self):
        super().__init__('DOCKING_NODE')

        ###### Initializing variables used in the code ###### 
        self.ultra_left=None
        self.ultra_right=None
        self.ebot_orientation = None
        self.callback_group = ReentrantCallbackGroup()
        ######################################################

        ####### Subscribers & Publishers #######
        self.ultra_sub = self.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', self.ultra_callback, 10)
        self.orientation_sub = self.create_subscription(Float32, 'orientation',self.orientation_callback, 10)
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)
        self.publisher_cmd_vel = self.create_publisher(Twist,'/cmd_vel',10)  
        #############################



        ####### Functions to be called only once #######         
        # self.reset_imu()                                    # Reset IMU data
        # self.reset_odom()                                   # Reset Odom
        #################################################
        self.is_docking = False
        self.dock_aligned = False
        self.rack_docked = False
        self.undocking = False
        self.orientation_docked = False
        self.rack_undocked = False
        self.dock_bool = False
        self.undock_bool = False

        self.robot_pose = [0,0,0]
        self.rack_yaw = 0.0
        
        ####### Create a main loop where the main logic of the code will be implemented #######
        self.controller_timer = self.create_timer(0.1, self.theme_logic)
        #######################################################################################

    def orientation_callback(self,msg):
        self.ebot_orientation=msg.data
        self.robot_pose[2] = self.ebot_orientation
        # self.get_logger().info(f'Orientation: {self.ebot_orientation}')

    def ultra_callback(self,msg):
        self.ultra_left= msg.data[4]
        self.ultra_right = msg.data[5]
        # self.get_logger().info(f'Ultrasonic Left: {self.ultra_left}; Ultrasonic Right: {self.ultra_right}')

    def reset_odom(self):
        self.get_logger().info('Resetting Odometry. Please wait...')
        self.reset_odom_ebot = self.create_client(Trigger, 'reset_odom')
        while not self.reset_odom_ebot.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/reset_odom service not available. Waiting for /reset_odom to become available.')

        self.request_odom_reset = Trigger.Request()
        self.odom_service_resp=self.reset_odom_ebot.call_async(self.request_odom_reset)
        rclpy.spin_until_future_complete(self, self.odom_service_resp)
        if(self.odom_service_resp.result().success== True):
            self.get_logger().info(self.odom_service_resp.result().message)
        else:
            self.get_logger().warn(self.odom_service_resp.result().message)

    def reset_imu(self):
        self.get_logger().info('Resetting IMU. Please wait...')
        self.reset_imu_ebot = self.create_client(Trigger, 'reset_imu')
        while not self.reset_imu_ebot.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/reset_imu service not available. Waiting for /reset_imu to become available.')

        request_imu_reset = Trigger.Request()
        self.imu_service_resp=self.reset_imu_ebot.call_async(request_imu_reset)
        rclpy.spin_until_future_complete(self, self.imu_service_resp)
        if(self.imu_service_resp.result().success== True):
            self.get_logger().info(self.imu_service_resp.result().message)
        else:
            self.get_logger().warn(self.imu_service_resp.result().message)

    def switch_eletromagent(self,relayState):
        self.get_logger().info('Changing state of the relay to '+str(relayState))
        self.trigger_usb_relay = self.create_client(RelaySw, 'usb_relay_sw')
        while not self.trigger_usb_relay.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('USB Trigger Service not available, waiting...')

        request_relay = RelaySw.Request()
        request_relay.relaychannel = True
        request_relay.relaystate = relayState
        self.usb_relay_service_resp=self.trigger_usb_relay.call_async(request_relay)
        rclpy.spin_until_future_complete(self, self.usb_relay_service_resp)
        if(self.usb_relay_service_resp.result().success== True):
            self.get_logger().info(self.usb_relay_service_resp.result().message)
        else:
            self.get_logger().warn(self.usb_relay_service_resp.result().message)

    def adjust_angle(self , pose):
        v_msg = Twist()
        
        if (pose == 0):
            self.l_limit = 4.71
            self.r_limit = 1.57
        else:
            self.l_limit = pose - 1.57
            self.r_limit = pose + 1.57

        if (not self.orientation_docked):

            if (pose ==0):

                if ((self.robot_pose[2]<1.57) and ((abs(self.robot_pose[2]-pose)>0.01))):
                    v_msg.linear.x = 0.0
                    v_msg.angular.z = -0.5
                    self.publisher_cmd_vel.publish(v_msg)
                elif ((self.robot_pose[2]>4.71) and ((abs(6.28-self.robot_pose[2])>0.01))):
                    v_msg.linear.x = 0.0
                    v_msg.angular.z = 0.5
                    self.publisher_cmd_vel.publish(v_msg)
                else:
                    v_msg.linear.x = 0.0
                    v_msg.angular.z = 0.0
                    self.publisher_cmd_vel.publish(v_msg)
                    self.orientation_docked = True
                    print("Angle Adjusted")

            else :

                if ((self.robot_pose[2]<self.r_limit)and(self.robot_pose[2]>pose)and(abs(self.robot_pose[2]-pose)>0.01)):
                    v_msg.linear.x = 0.0
                    v_msg.angular.z = -0.5
                    self.publisher_cmd_vel.publish(v_msg)
                elif ((self.robot_pose[2]>self.l_limit)and(self.robot_pose[2]<pose)and(abs(self.robot_pose[2]-pose)>0.01)):
                    v_msg.linear.x = 0.0
                    v_msg.angular.z = 0.5
                    self.publisher_cmd_vel.publish(v_msg)
                else:
                    v_msg.linear.x = 0.0
                    v_msg.angular.z = 0.0
                    self.publisher_cmd_vel.publish(v_msg)
                    self.orientation_docked = True
                    print("Angle Adjusted")


    
    def theme_logic(self):
        '''
        Main function where the logic will be written
        '''
        v_msg = Twist()
        print(self.robot_pose[2])
        print(self.ultra_left , self.ultra_right)
        if (self.usrleft_value and self.usrright_value):
            self.is_docking = True

        if self.is_docking :

            #self.adjust_angle(self.rack_yaw)
            if self.orientation_docked:
                if ((abs(self.ultra_left)<0.1)and(abs(self.ultra_right)<0.1)):
                    v_msg.linear.x = 0.0
                    v_msg.angular.z = 0.0
                    self.publisher_cmd_vel.publish(v_msg)
                    self.switch_eletromagent(1)
                    print("rack docked successfully...")


                    self.rack_docked = True
                    self.dock_aligned = True
                    self.is_docking = False



                else:
                    if ((abs(self.ultra_left - self.ultra_right)>0.005)and(self.ultra_left>self.ultra_right)):
                        v_msg.linear.x = 0.0
                        v_msg.angular.z = 0.5
                        self.publisher_cmd_vel.publish(v_msg)


                    if ((abs(self.ultra_left - self.ultra_right)>0.005)and(self.ultra_left<self.ultra_right)):
                        v_msg.linear.x = 0.0
                        v_msg.angular.z = -0.5
                        self.publisher_cmd_vel.publish(v_msg)


                    if ((abs(self.ultra_left - self.ultra_right)<0.005)):
                        v_msg.linear.x = -0.4
                        v_msg.angular.z = 0.0
                        self.publisher_cmd_vel.publish(v_msg)

        if self.undocking :
            self.orientation_docked = False
            #self.adjust_angle(3.14)
            if self.orientation_docked:
                print("Rack Undocked")
                self.switch_eletromagent(0)
                self.undocking = False
                self.rack_undocked = True
                self.dock_aligned = True

        pass

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

            

        

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ThemeImplementation()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("###### Keyboard interrupt detected. Closing script. ######")

if __name__ == '__main__':
    main()