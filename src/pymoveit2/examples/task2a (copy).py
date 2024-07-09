#!/usr/bin/env python3

from threading import Thread
from os import path
import rclpy
import sys
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from std_msgs.msg import Float64MultiArray
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink



class MinimalSubscriber(Node):

    def __init__ (self):
        super().__init__('box_coordinates_subscriber')
        self.create_subscription(Float64MultiArray,'/box1_coordinates',self.box1_cc_finder,10)
        self.create_subscription(Float64MultiArray,'/box2_coordinates',self.box2_cc_finder,10)
        self.create_subscription(Float64MultiArray,'/box3_coordinates',self.box3_cc_finder,10)
        self.create_subscription(Float64MultiArray,'/ids',self.id_array_finder,10)
        
        self.box1_coordinates = None
        self.box2_coordinates = None
        self.box3_coordinates = None
        self.id_list = None

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

    def id_array_finder(self , msg):
        if self.id_list is None :
            self.id_list = msg.data
            print(f'ids : {self.id_list}')
    
    def move_arm(self):

        node = Node("ex_pose_goal")

        position = self.box1_coordinates
        quat_xyzw = [0.5, 0.5, 0.5, 0.5]
        cartesian = False
        drop =[-0.37, 0.12, 0.397]


        

        moveit2 = MoveIt2(
               
            node = node,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            
        )
        
        

        self.get_logger().info(
            f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
        moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
        moveit2.wait_until_executed()

        gripper_control = self.create_client(AttachLink, '/GripperMagnetON')
        while not gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')
        
        req = AttachLink.Request()
        req.model1_name =  'box1'      
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'
        
        gripper_control.call_async(req)
        time.sleep(5)

        self.get_logger().info(
            f"Moving to {{position: {list(drop)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
        moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
        moveit2.wait_until_executed()

        gripper_control = self.create_client(DetachLink, '/GripperMagnetON')
        while not gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')
        
        req = AttachLink.Request()
        req.model1_name =  'box1'      
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'
        
        gripper_control.call_async(req)
        time.sleep(5)
        

def main():
    
    rclpy.init(args=sys.argv)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()