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
        ######################################################

        ####### Subscribers & Publishers #######
        self.ultra_sub = self.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', self.ultra_callback, 10)
        self.orientation_sub = self.create_subscription(Float32, 'orientation',self.orientation_callback, 10)
        self.dock_bool_sub = self.create_subscription(Bool , 'dock_bool' , self.dock_bool_callback , 10)
        self.undock_bool_sub = self.create_subscription(Bool , 'undock_bool' , self.undock_bool_callback , 10)
        self.rack_docked_pub = self.create_publisher(Bool, 'rack_docked_bool' , 10 )
        self.rack_undocked_pub = self.create_publisher(Bool, 'rack_undocked_bool' , 10 )
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
        self.rack_yaw = 3.14
        
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

    def theme_logic(self):
        '''
        Main function where the logic will be written
        '''
        v_msg = Twist()

        if self.dock_bool:
            self.is_docking = True

        if self.undock_bool:
            self.undocking = True

        if self.is_docking :

           
            if ((self.robot_pose[2]<self.rack_yaw)and(abs(self.robot_pose[2]-self.rack_yaw)>0.01)and(self.orientation_docked == False)):
                v_msg.linear.x = 0.0
                v_msg.angular.z = 0.2
                self.publisher_cmd_vel.publish(v_msg)
            if ((self.robot_pose[2]>self.rack_yaw)and(abs(self.robot_pose[2]-self.rack_yaw)>0.01)and(self.orientation_docked == False)):
                v_msg.linear.x = 0.0
                v_msg.angular.z = -0.2
                self.publisher_cmd_vel.publish(v_msg)
            if ((abs(self.robot_pose[2]-self.rack_yaw)<0.01)and(self.orientation_docked == False)):
                v_msg.linear.x = 0.0
                v_msg.angular.z = 0.0
                self.publisher_cmd_vel.publish(v_msg)
                self.orientation_docked = True
                print("Angle adjusted")

            if ((abs(self.ultra_left)<0.1)and(abs(self.ultra_right)<0.1)):
                v_msg.linear.x = 0.0
                v_msg.angular.z = 0.0
                self.publisher_cmd_vel.publish(v_msg)
                switch_eletromagent(1)
                print("rack docked successfully...")
                

                self.rack_docked = True
                self.dock_aligned = True
                self.is_docking = False
                
                
                
            else:
                if ((abs(self.ultra_left - self.ultra_right)>0.005)and(self.ultra_left>self.ultra_right)and(self.orientation_docked == True)):
                    v_msg.linear.x = 0.0
                    v_msg.angular.z = 0.5
                    self.publisher_cmd_vel.publish(v_msg)
                    

                if ((abs(self.ultra_left - self.ultra_right)>0.005)and(self.ultra_left<self.ultra_right)and(self.orientation_docked == True)):
                    v_msg.linear.x = 0.0
                    v_msg.angular.z = -0.5
                    self.publisher_cmd_vel.publish(v_msg)
                    

                if ((abs(self.ultra_left - self.ultra_right)<0.005)and(self.orientation_docked == True)):
                    v_msg.linear.x = -0.15
                    v_msg.angular.z = 0.0
                    self.publisher_cmd_vel.publish(v_msg)


        rack_docked_bool = Bool()
        rack_docked_bool.data = self.rack_docked
        self.rack_docked_pub.publish(rack_docked_bool)
        self.get_logger().info(f'Publishing: Rack_docked = {rack_docked_bool.data}')



        if self.undocking :

            

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
                switch_eletromagent(0)
                self.undocking = False
                self.rack_undocked = True
                self.dock_aligned = True
            
        rack_undocked_bool = Bool()
        rack_undocked_bool.data = self.rack_undocked
        self.rack_undocked_pub.publish(rack_undocked_bool)
        self.get_logger().info(f'Publishing: Rack_undocked = {rack_undocked_bool.data}')   

            


        pass


            

        

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