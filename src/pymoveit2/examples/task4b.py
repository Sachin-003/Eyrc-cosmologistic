#!/usr/bin/env python3

from math import dist
from platform import node
from threading import Thread
from os import path
from turtle import distance, position
import rclpy
import sys
import math
import time
from ur_msgs.srv import SetIO
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from std_msgs.msg import Float64MultiArray
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TwistStamped
#from linkattacher_msgs.srv import AttachLink
#from linkattacher_msgs.srv import DetachLink
import tf2_ros
import numpy as np
import transformations as tf
from tf2_ros import TransformException
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

class MinimalSubscriber(Node):

    def __init__ (self):
        super().__init__('box_coordinates_subscriber')
        self.create_subscription(Float64MultiArray,'/box1_coordinates',self.box1_cc_finder,10)
        self.create_subscription(Float64MultiArray,'/box2_coordinates',self.box2_cc_finder,10)
        self.create_subscription(Float64MultiArray,'/box3_coordinates',self.box3_cc_finder,10)
        self.create_subscription(Float64MultiArray,'/ids',self.id_array_finder,10)
        self.create_subscription(Float64MultiArray,'/yaw',self.angle_finder,10)
        self.callback_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(0.4, self.move_arm)
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self) 
        self.__twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        #self.gripper_control = self.create_client(AttachLink, '/GripperMagnetON')
        #self.gripper_control_off = self.create_client(DetachLink, '/GripperMagnetOFF')
        self.__contolMSwitch = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.box1_coordinates = None
        self.box2_coordinates = None
        self.box3_coordinates = None
        self.id_list = None
        self.angle_list = None
        self.x_align = False
        self.y_align = False
        self.z_align = False
        self.in_correct_joints = False
        self.initial_pose = False
        self.in_front = False
        self.look_up = False
        self.data_fetched  =  False

    def box1_cc_finder(self , msg):
        if self.box1_coordinates is None :
            self.box1_coordinates = msg.data
            print(f'box1 c arr : {self.box1_coordinates}')

    def box2_cc_finder(self , msg):
        if self.box2_coordinates is None :
            self.box2_coordinates = msg.data
            print(f'box2 c arr : {self.box2_coordinates}')
            

    def box3_cc_finder(self , msg):
        if self.box3_coordinates is None :
            self.box3_coordinates = msg.data
            print(f'box3 c arr : {self.box3_coordinates}')

    def id_array_finder(self , msg):
        if self.id_list is None :
            self.id_list = msg.data
            print(f'ids : {self.id_list}')

    def angle_finder(self,msg ):
        if self.angle_list is None :
            self.angle_list = msg.data
            print(f"angles : {self.angle_list}")

    def gripper_call(self, state):
        '''
        based on the state given as i/p the service is called to activate/deactivate
        pin 16 of TCP in UR5
        i/p: node, state of pin:Bool
        o/p or return: response from service call
        '''
        gripper_control = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF Tool service not available, waiting again...')
        req         = SetIO.Request()
        req.fun     = 1
        req.pin     = 16
        req.state   = float(state)
        gripper_control.call_async(req)
        return state


    def move_arm(self):


        if (self.box1_coordinates and self.box2_coordinates and self.box3_coordinates and self.id_list and self.angle_list):
            self.data_fetched = True
        else:
            print("Fetching coordinates>>>>>>>>")

        if self.data_fetched:

            switchParam = SwitchController.Request()
            f2 = 'tool0'
            f3 = 'base_link'
            while (not self.look_up):
                try:
                    ts = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                    tool0_x = ts.transform.translation.x
                    tool0_y = ts.transform.translation.y
                    tool0_z = ts.transform.translation.z
                    print("lookup done>>>>>>>")
                    self.look_up = True

                except:
                    print("Could not lookup>>>>>>>")
            boxes_coordinates = [self.box1_coordinates,self.box2_coordinates, self.box3_coordinates]

            vx = -0.2
            vy = -0.2
            vz = -0.2

            dist_x = tool0_x - boxes_coordinates[0][0]
            dist_y = tool0_y - boxes_coordinates[0][1]
            dist_z = tool0_z - boxes_coordinates[0][2]

            if dist_x<0: 
                vx = 0.2

            if dist_y<0: 
                vy = 0.2

            if dist_z<0: 
                vz = 0.2

            initial_pose = [0.0 , -2.3911 , 2.40855 , -3.14159 , -1.58825 , 3.14159]
            drop_pose = [0.0 , -1.81514 , -1.309 , -3.14159 , -1.58825 , 3.14159]

            drop_pose_1 = [0.0 ,-2.26893, -0.837758 , -3.14159 , -1.58825 , 3.14159]

            if self.angle_list[0]<1 :
                initial_pose[0] = -1.57

            if self.angle_list[0]>3 :
                initial_pose[0] = 1.57

            if self.angle_list[0]>1 and self.angle_list[0]<3 :
                self.in_correct_joints = True
                self.in_front = True

            switchParam.activate_controllers = ["scaled_joint_trajectory_controller"] # for normal use of moveit
            switchParam.deactivate_controllers = ["forward_position_controller"] # for servoing
            switchParam.strictness = 2
            switchParam.start_asap = False

            # calling control manager service after checking its availability
            while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f"Service control Manager is not yet available...")
            self.__contolMSwitch.call_async(switchParam)
            print("[CM]: Switching Complete")

            while (not self.in_correct_joints):

                node1 = Node('initial_pose_1')
                moveit1 = MoveIt2(
                    node = node1,
                    joint_names=ur5.joint_names(),
                    base_link_name=ur5.base_link_name(),
                    end_effector_name=ur5.end_effector_name(),
                    group_name=ur5.MOVE_GROUP_ARM,
                    callback_group=self.callback_group
                    )

                self.get_logger().info(f"Moving to {{joint_positions: {list(initial_pose)}}}")
                moveit1.move_to_configuration(initial_pose)
                self.in_correct_joints = True
                time.sleep(4)

            switchParam.deactivate_controllers = ["scaled_joint_trajectory_controller"]
            switchParam.activate_controllers = ["forward_position_controller"] 
            switchParam.strictness = 2
            switchParam.start_asap = False
            
            # calling control manager service after checking its availability
            while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f"Service control Manager is not yet available...")
            self.__contolMSwitch.call_async(switchParam)
            print("[CM]: Switching Complete")
            while (abs(dist_z)>0.01):
                self.look_up = False

                while (not self.look_up):
                    try:
                        t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                        tool0_x = t.transform.translation.x
                        tool0_y = t.transform.translation.y
                        tool0_z = t.transform.translation.z
                        self.look_up = True

                    except:
                        print("Could not lookup>>>>>>>2")


                dist_x = tool0_x - boxes_coordinates[0][0]
                dist_y = tool0_y - boxes_coordinates[0][1]

                if abs(dist_x)<0.1:
                    vx = 0.0
                if abs(dist_y)<0.1:
                    vy = 0.0

                __twist_msg = TwistStamped()
                current_time = self.get_clock().now()
                __twist_msg.header.frame_id = ur5.base_link_name()
                __twist_msg.header.stamp = current_time.to_msg()
                __twist_msg.twist.linear.x = vx
                __twist_msg.twist.linear.y = vy
                __twist_msg.twist.linear.z = vz
                __twist_msg.twist.angular.x = 0.0
                __twist_msg.twist.angular.y = 0.0
                __twist_msg.twist.angular.z = 0.0
                self.__twist_pub.publish(__twist_msg)
                dist_z = tool0_z - boxes_coordinates[0][2]
                time.sleep(0.2)



            while (abs(dist_y)>0.02):
                self.look_up = False
                while (not self.look_up):
                    try:
                        t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                        tool0_x = t.transform.translation.x
                        tool0_y = t.transform.translation.y
                        tool0_z = t.transform.translation.z
                        self.look_up = True

                    except:
                        print("Could not lookup>>>>>>>2")

                dist_y = tool0_y - boxes_coordinates[0][1]
                if dist_y<0 :
                    vy = 0.2
                if dist_y>0 :
                    vy = -0.2

                __twist_msg = TwistStamped()
                current_time = self.get_clock().now()
                __twist_msg.header.frame_id = ur5.base_link_name()
                __twist_msg.header.stamp = current_time.to_msg()
                __twist_msg.twist.linear.x = 0.0
                __twist_msg.twist.linear.y = vy
                __twist_msg.twist.linear.z = 0.0
                __twist_msg.twist.angular.x = 0.0
                __twist_msg.twist.angular.y = 0.0
                __twist_msg.twist.angular.z = 0.0
                self.__twist_pub.publish(__twist_msg)
                time.sleep(0.2)



            while (abs(dist_x)>0.01):
                self.look_up = False
                while (not self.look_up):
                    try:
                        t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                        tool0_x = t.transform.translation.x
                        tool0_y = t.transform.translation.y
                        tool0_z = t.transform.translation.z
                        self.look_up = True

                    except:
                        print("Could not lookup>>>>>>>2")
                dist_x = tool0_x - boxes_coordinates[0][0]
                if dist_x<0 : 
                    vx = 0.2
                if dist_x>0 :
                    vx  = -0.2

                __twist_msg = TwistStamped()
                current_time = self.get_clock().now()
                __twist_msg.header.frame_id = ur5.base_link_name()
                __twist_msg.header.stamp = current_time.to_msg()
                __twist_msg.twist.linear.x = vx
                __twist_msg.twist.linear.y = 0.0
                __twist_msg.twist.linear.z = 0.0
                __twist_msg.twist.angular.x = 0.0
                __twist_msg.twist.angular.y = 0.0
                __twist_msg.twist.angular.z = 0.0
                self.__twist_pub.publish(__twist_msg)

                time.sleep(0.2)

            gripper_call(1)
            self.link_attached = True

            if self.in_front : 

                while (abs(dist_x)<0.15):

                    self.look_up = False
                    while (not self.look_up):
                        try:
                            t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                            tool0_x = t.transform.translation.x
                            tool0_y = t.transform.translation.y
                            tool0_z = t.transform.translation.z
                            self.look_up = True

                        except:
                            print("Could not lookup>>>>>>>2")
                    dist_x = tool0_x - boxes_coordinates[0][0]
                    if dist_x<0 : 
                        vx = -0.2
                    if dist_x>0 :
                        vx  = 0.2

                    __twist_msg = TwistStamped()
                    current_time = self.get_clock().now()
                    __twist_msg.header.frame_id = ur5.base_link_name()
                    __twist_msg.header.stamp = current_time.to_msg()
                    __twist_msg.twist.linear.x = vx
                    __twist_msg.twist.linear.y = 0.0
                    __twist_msg.twist.linear.z = 0.0
                    __twist_msg.twist.angular.x = 0.0
                    __twist_msg.twist.angular.y = 0.0
                    __twist_msg.twist.angular.z = 0.0
                    self.__twist_pub.publish(__twist_msg)
                    time.sleep(0.2)

            else:

                while (abs(dist_y)<0.15):
                    self.look_up = False
                    while (not self.look_up):
                        try:
                            t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                            tool0_x = t.transform.translation.x
                            tool0_y = t.transform.translation.y
                            tool0_z = t.transform.translation.z
                            self.look_up = True

                        except:
                            print("Could not lookup>>>>>>>2")

                    dist_y = tool0_y - boxes_coordinates[0][1]
                    if self.angle_list[0]<1:
                        vy = 0.2
                    if self.angle_list[0]>3 :
                        vy = -0.2

                    __twist_msg = TwistStamped()
                    current_time = self.get_clock().now()
                    __twist_msg.header.frame_id = ur5.base_link_name()
                    __twist_msg.header.stamp = current_time.to_msg()
                    __twist_msg.twist.linear.x = 0.0
                    __twist_msg.twist.linear.y = vy
                    __twist_msg.twist.linear.z = 0.0
                    __twist_msg.twist.angular.x = 0.0
                    __twist_msg.twist.angular.y = 0.0
                    __twist_msg.twist.angular.z = 0.0
                    self.__twist_pub.publish(__twist_msg)
                    time.sleep(0.2)

                if self.angle_list[0]<1:
                    initial_pose[0] = -1.57
                else :
                    initial_pose[0] = 1.57




                switchParam.activate_controllers = ["scaled_joint_trajectory_controller"]
                switchParam.deactivate_controllers = ["forward_position_controller"] 
                switchParam.strictness = 2
                switchParam.start_asap = False

                # calling control manager service after checking its availability
                while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
                    self.get_logger().warn(f"Service control Manager is not yet available...")
                self.__contolMSwitch.call_async(switchParam)
                print("[CM]: Switching Complete")


                self.in_correct_joints = False
                while (not self.in_correct_joints):

                    node3 = Node('initial_pose_3')
                    moveit3 = MoveIt2(
                        node = node3,
                        joint_names=ur5.joint_names(),
                        base_link_name=ur5.base_link_name(),
                        end_effector_name=ur5.end_effector_name(),
                        group_name=ur5.MOVE_GROUP_ARM,
                        callback_group=self.callback_group
                        )

                    self.get_logger().info(f"Moving to {{joint_positions: {list(initial_pose)}}}")
                    moveit3.move_to_configuration(initial_pose)
                    self.in_correct_joints = True
                    time.sleep(4)



            self.in_correct_joints = False

            while (not self.in_correct_joints):

                node4 = Node('drop_pose')
                moveit4 = MoveIt2(
                    node = node4,
                    joint_names=ur5.joint_names(),
                    base_link_name=ur5.base_link_name(),
                    end_effector_name=ur5.end_effector_name(),
                    group_name=ur5.MOVE_GROUP_ARM,
                    callback_group=self.callback_group
                    )

                self.get_logger().info(f"Moving to {{joint_positions: {list(drop_pose)}}}")
                moveit4.move_to_configuration(drop_pose_1)
                self.in_correct_joints = True
                time.sleep(4)

            gripper_call(0)

            self.in_correct_joints = False
            initial_pose[0] = 0.0

            while (not self.in_correct_joints):

                node5 = Node('initial_pose_5')
                moveit5 = MoveIt2(
                    node = node5,
                    joint_names=ur5.joint_names(),
                    base_link_name=ur5.base_link_name(),
                    end_effector_name=ur5.end_effector_name(),
                    group_name=ur5.MOVE_GROUP_ARM,
                    callback_group=self.callback_group
                    )

                self.get_logger().info(f"Moving to {{joint_positions: {list(initial_pose)}}}")
                moveit5.move_to_configuration(initial_pose)
                self.in_correct_joints = True
                time.sleep(4)

            vx = -0.2
            vy = -0.2
            vz = -0.2

            self.look_up = False

            while (not self.look_up):
                try:
                    ts = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                    tool0_x = ts.transform.translation.x
                    tool0_y = ts.transform.translation.y
                    tool0_z = ts.transform.translation.z
                    print("lookup done>>>>>>>")
                    self.look_up = True

                except:
                    print("Could not lookup>>>>>>>")

            dist_x = tool0_x - boxes_coordinates[1][0]
            dist_y = tool0_y - boxes_coordinates[1][1]
            dist_z = tool0_z - boxes_coordinates[1][2]

            if dist_x<0: 
                vx = 0.2

            if dist_y<0: 
                vy = 0.2

            if dist_z<0: 
                vz = 0.2

            self.in_correct_joints = False
            self.in_front = False
            if self.angle_list[1]<1 :
                initial_pose[0] = -1.57

            if self.angle_list[1]>3 :
                initial_pose[0] = 1.57

            if self.angle_list[1]>1 and self.angle_list[1]<3 :
                self.in_correct_joints = True
                self.in_front = True

            while (not self.in_correct_joints):

                node6 = Node('initial_pose_6')
                moveit6 = MoveIt2(
                    node = node6,
                    joint_names=ur5.joint_names(),
                    base_link_name=ur5.base_link_name(),
                    end_effector_name=ur5.end_effector_name(),
                    group_name=ur5.MOVE_GROUP_ARM,
                    callback_group=self.callback_group
                    )

                self.get_logger().info(f"Moving to {{joint_positions: {list(initial_pose)}}}")
                moveit6.move_to_configuration(initial_pose)
                self.in_correct_joints = True
                time.sleep(4)


            switchParam.deactivate_controllers = ["scaled_joint_trajectory_controller"]
            switchParam.activate_controllers = ["forward_position_controller"] 
            switchParam.strictness = 2
            switchParam.start_asap = False
            # calling control manager service after checking its availability
            while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f"Service control Manager is not yet available...")
            self.__contolMSwitch.call_async(switchParam)
            print("[CM]: Switching Complete")

            while (abs(dist_z)>0.01):
                self.look_up = False

                while (not self.look_up):
                    try:
                        t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                        tool0_x = t.transform.translation.x
                        tool0_y = t.transform.translation.y
                        tool0_z = t.transform.translation.z
                        self.look_up = True

                    except:
                        print("Could not lookup>>>>>>>2")
                dist_x = tool0_x - boxes_coordinates[1][0]
                dist_y = tool0_y - boxes_coordinates[1][1]

                if abs(dist_x)<0.1:
                    vx = 0.0
                if abs(dist_y)<0.1:
                    vy = 0.0

                __twist_msg = TwistStamped()
                current_time = self.get_clock().now()
                __twist_msg.header.frame_id = ur5.base_link_name()
                __twist_msg.header.stamp = current_time.to_msg()
                __twist_msg.twist.linear.x = vx
                __twist_msg.twist.linear.y = vy
                __twist_msg.twist.linear.z = vz
                __twist_msg.twist.angular.x = 0.0
                __twist_msg.twist.angular.y = 0.0
                __twist_msg.twist.angular.z = 0.0
                self.__twist_pub.publish(__twist_msg)
                dist_z = tool0_z - boxes_coordinates[1][2]
                time.sleep(0.2)

            while (abs(dist_y)>0.02):
                self.look_up = False

                while (not self.look_up):
                    try:
                        t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                        tool0_x = t.transform.translation.x
                        tool0_y = t.transform.translation.y
                        tool0_z = t.transform.translation.z
                        self.look_up = True

                    except:
                        print("Could not lookup>>>>>>>2")

                dist_y = tool0_y - boxes_coordinates[1][1]
                if dist_y<0 :
                    vy = 0.2
                if dist_y>0 :
                    vy = -0.2

                __twist_msg = TwistStamped()
                current_time = self.get_clock().now()
                __twist_msg.header.frame_id = ur5.base_link_name()
                __twist_msg.header.stamp = current_time.to_msg()
                __twist_msg.twist.linear.x = 0.0
                __twist_msg.twist.linear.y = vy
                __twist_msg.twist.linear.z = 0.0
                __twist_msg.twist.angular.x = 0.0
                __twist_msg.twist.angular.y = 0.0
                __twist_msg.twist.angular.z = 0.0
                self.__twist_pub.publish(__twist_msg)
                time.sleep(0.2)

            while (abs(dist_x)>0.01):
                self.look_up = False

                while (not self.look_up):
                    try:
                        t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                        tool0_x = t.transform.translation.x
                        tool0_y = t.transform.translation.y
                        tool0_z = t.transform.translation.z
                        self.look_up = True

                    except:
                        print("Could not lookup>>>>>>>2")
                dist_x = tool0_x - boxes_coordinates[1][0]
                if dist_x<0 : 
                    vx = 0.2
                if dist_x>0 :
                    vx  = -0.2

                __twist_msg = TwistStamped()
                current_time = self.get_clock().now()
                __twist_msg.header.frame_id = ur5.base_link_name()
                __twist_msg.header.stamp = current_time.to_msg()
                __twist_msg.twist.linear.x = vx
                __twist_msg.twist.linear.y = 0.0
                __twist_msg.twist.linear.z = 0.0
                __twist_msg.twist.angular.x = 0.0
                __twist_msg.twist.angular.y = 0.0
                __twist_msg.twist.angular.z = 0.0
                self.__twist_pub.publish(__twist_msg)

                time.sleep(0.2)

            gripper_call(1)

            if self.in_front : 

                while (abs(dist_x)<0.15):

                    self.look_up = False

                    while (not self.look_up):
                        try:
                            t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                            tool0_x = t.transform.translation.x
                            tool0_y = t.transform.translation.y
                            tool0_z = t.transform.translation.z
                            self.look_up = True

                        except:
                            print("Could not lookup>>>>>>>2")
                    dist_x = tool0_x - boxes_coordinates[1][0]


                    __twist_msg = TwistStamped()
                    current_time = self.get_clock().now()
                    __twist_msg.header.frame_id = ur5.base_link_name()
                    __twist_msg.header.stamp = current_time.to_msg()
                    __twist_msg.twist.linear.x = -0.2
                    __twist_msg.twist.linear.y = 0.0
                    __twist_msg.twist.linear.z = 0.0
                    __twist_msg.twist.angular.x = 0.0
                    __twist_msg.twist.angular.y = 0.0
                    __twist_msg.twist.angular.z = 0.0
                    self.__twist_pub.publish(__twist_msg)
                    time.sleep(0.2)

                self.in_correct_joints = False
                while (not self.in_correct_joints):

                    node11 = Node('drop_pose_xx')
                    moveit11 = MoveIt2(
                        node = node11,
                        joint_names=ur5.joint_names(),
                        base_link_name=ur5.base_link_name(),
                        end_effector_name=ur5.end_effector_name(),
                        group_name=ur5.MOVE_GROUP_ARM,
                        callback_group=self.callback_group
                        )

                    self.get_logger().info(f"Moving to {{joint_positions: {list(initial_pose)}}}")
                    moveit11.move_to_configuration(drop_pose)
                    self.in_correct_joints = True
                    time.sleep(4)


            else:

                while (abs(dist_y)<0.15):
                    self.look_up = False

                    while (not self.look_up):
                        try:
                            t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                            tool0_x = t.transform.translation.x
                            tool0_y = t.transform.translation.y
                            tool0_z = t.transform.translation.z
                            self.look_up = True
    
                        except:
                            print("Could not lookup>>>>>>>2")

                    dist_y = tool0_y - boxes_coordinates[1][1]
                    if self.angle_list[1]<1:
                        vy = 0.2
                    if self.angle_list[1]>3 :
                        vy = -0.2

                    __twist_msg = TwistStamped()
                    current_time = self.get_clock().now()
                    __twist_msg.header.frame_id = ur5.base_link_name()
                    __twist_msg.header.stamp = current_time.to_msg()
                    __twist_msg.twist.linear.x = 0.0
                    __twist_msg.twist.linear.y = vy
                    __twist_msg.twist.linear.z = 0.0
                    __twist_msg.twist.angular.x = 0.0
                    __twist_msg.twist.angular.y = 0.0
                    __twist_msg.twist.angular.z = 0.0
                    self.__twist_pub.publish(__twist_msg)
                    time.sleep(0.2)

                if self.angle_list[1]<1:
                    initial_pose[0] = -1.57
                else :
                    initial_pose[0] = 1.57


                switchParam.activate_controllers = ["scaled_joint_trajectory_controller"]
                switchParam.deactivate_controllers = ["forward_position_controller"] 
                switchParam.strictness = 2
                switchParam.start_asap = False
                # calling control manager service after checking its availability
                while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
                    self.get_logger().warn(f"Service control Manager is not yet available...")
                self.__contolMSwitch.call_async(switchParam)
                print("[CM]: Switching Complete")


                self.in_correct_joints = False
                while (not self.in_correct_joints):

                    node7 = Node('initial_pose_7')
                    moveit7 = MoveIt2(
                        node = node7,
                        joint_names=ur5.joint_names(),
                        base_link_name=ur5.base_link_name(),
                        end_effector_name=ur5.end_effector_name(),
                        group_name=ur5.MOVE_GROUP_ARM,
                        callback_group=self.callback_group
                        )

                    self.get_logger().info(f"Moving to {{joint_positions: {list(initial_pose)}}}")
                    moveit7.move_to_configuration(initial_pose)
                    self.in_correct_joints = True
                    time.sleep(4)

            self.in_correct_joints = False

            switchParam.activate_controllers = ["scaled_joint_trajectory_controller"]
            switchParam.deactivate_controllers = ["forward_position_controller"] 
            switchParam.strictness = 2
            switchParam.start_asap = False
            # calling control manager service after checking its availability
            while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f"Service control Manager is not yet available...")
            self.__contolMSwitch.call_async(switchParam)
            print("[CM]: Switching Complete")

            while (not self.in_correct_joints):

                node9 = Node('drop_pose_2')
                moveit9 = MoveIt2(
                    node = node9,
                    joint_names=ur5.joint_names(),
                    base_link_name=ur5.base_link_name(),
                    end_effector_name=ur5.end_effector_name(),
                    group_name=ur5.MOVE_GROUP_ARM,
                    callback_group=self.callback_group
                    )

                self.get_logger().info(f"Moving to {{joint_positions: {list(drop_pose)}}}")
                moveit9.move_to_configuration(drop_pose)
                self.in_correct_joints = True
                time.sleep(4)

            gripper_call(0)

            self.in_correct_joints = False
            initial_pose[0] = 0.0
            while (not self.in_correct_joints):

                node10 = Node('initial_pose_10')
                moveit10 = MoveIt2(
                    node = node10,
                    joint_names=ur5.joint_names(),
                    base_link_name=ur5.base_link_name(),
                    end_effector_name=ur5.end_effector_name(),
                    group_name=ur5.MOVE_GROUP_ARM,
                    callback_group=self.callback_group
                    )

                self.get_logger().info(f"Moving to {{joint_positions: {list(initial_pose)}}}")
                moveit10.move_to_configuration(initial_pose)
                self.in_correct_joints = True
                time.sleep(4)

        
         
def main():
    
    rclpy.init(args=sys.argv)
    minimal_subscriber = MinimalSubscriber()
    executor = MultiThreadedExecutor()
    executor.add_node(minimal_subscriber)
    executor.spin()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()