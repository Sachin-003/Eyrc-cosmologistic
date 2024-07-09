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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool 
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TwistStamped
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
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
        self.create_subscription(Bool , 'arm_flag', self.is_racks_placed,10)
        self.callback_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(0.4, self.move_arm)
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self) 
        self.__twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.gripper_control = self.create_client(AttachLink, '/GripperMagnetON')
        self.gripper_control_off = self.create_client(DetachLink, '/GripperMagnetOFF')
        
        self.box1_coordinates = None
        self.box2_coordinates = None
        self.box3_coordinates = None
        self.id_list = None
        
        self.angle_list = None
        self.x_align = False
        self.y_align = False
        self.z_align = False
        self.racks_placed = False
        self.in_correct_joints = False
        self.initial_pose = False
        self.in_front = False

    def is_racks_placed(self , msg):
        if not self.racks_placed :
            self.racks_placed = msg.data
            print(self.racks_placed)
        else:
            print("Bool not found")

    def box1_cc_finder(self , msg):
        if (self.box1_coordinates is None) and (self.racks_placed):
            self.box1_coordinates = msg.data
            print(f'box1 c arr : {self.box1_coordinates}')
        else:
            print(f'box1 c arr : {self.box1_coordinates}')

    def box2_cc_finder(self , msg):
        if (self.box2_coordinates is None)and(self.racks_placed) :
            self.box2_coordinates = msg.data
            print(f'box2 c arr : {self.box2_coordinates}')
        else:
            print(f'box2 c arr : {self.box2_coordinates}')

    def box3_cc_finder(self , msg):
        if (self.box3_coordinates is None)and(self.racks_placed) :
            self.box3_coordinates = msg.data
        else:
            print(f'box3 c arr : {self.box3_coordinates}')

    def id_array_finder(self , msg):
        if (self.id_list is None) and(self.racks_placed) :
            self.id_list = msg.data
            print(f'ids : {self.id_list}')
        else:
            print(f'ids : {self.id_list}')

    def angle_finder(self,msg ):
        if (self.angle_list is None)and(self.racks_placed):
            self.angle_list = msg.data
            print(f"angles : {self.angle_list}")
        else:
            print(f"angles : {self.angle_list}")

    
    def move_arm(self):

        if (self.racks_placed) and (self.box1_coordinates is not None) :
            print("its time to move arm.")
            f2 = 'tool0'
            f3 = 'base_link'
            ts = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
            tool0_x = ts.transform.translation.x
            tool0_y = ts.transform.translation.y
            tool0_z = ts.transform.translation.z
        
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

            while (abs(dist_z)>0.01):
                t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                tool0_x = t.transform.translation.x
                tool0_y = t.transform.translation.y
                tool0_z = t.transform.translation.z
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
                t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                tool0_x = t.transform.translation.x
                tool0_y = t.transform.translation.y
                tool0_z = t.transform.translation.z

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
                t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                tool0_x = t.transform.translation.x
                tool0_y = t.transform.translation.y
                tool0_z = t.transform.translation.z
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

            while not self.gripper_control.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('EEF service not available, waiting again...')

            box_id = int(self.id_list[0])
            print(f"Box id : {box_id}")

            req = AttachLink.Request()
            req.model1_name =  f'box{box_id}'     
            req.link1_name  = 'link'       
            req.model2_name =  'ur5'       
            req.link2_name  = 'wrist_3_link'  

            self.gripper_control.call_async(req)
            time.sleep(2)
            
            
            self.link_attached = True

            if self.in_front : 

                while (abs(dist_x)<0.15):

                    t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                    tool0_x = t.transform.translation.x
                    tool0_y = t.transform.translation.y
                    tool0_z = t.transform.translation.z
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
                    t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                    tool0_x = t.transform.translation.x
                    tool0_y = t.transform.translation.y
                    tool0_z = t.transform.translation.z

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

            while not self.gripper_control_off.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('EEF service not available, waiting again...')

            box_id = int(self.id_list[0])
            print(f'Box_id : {box_id}')
            req = DetachLink.Request()
            req.model1_name =  f'box{box_id}'     
            req.link1_name  = 'link'       
            req.model2_name =  'ur5'       
            req.link2_name  = 'wrist_3_link'  

            self.gripper_control_off.call_async(req)

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