#!/usr/bin/env python3
from cgitb import grey
from email.mime import image
from turtle import color
from threading import Thread
import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
import time

from tf2_ros import TransformException
from geometry_msgs.msg import Pose
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink

def calculate_rectangle_area(coordinates):
   
    width = math.sqrt(((coordinates[0][0] - coordinates[1][0]) ** 2) + ((coordinates[0][1] - coordinates[1][1]) ** 2))
    height = math.sqrt(((coordinates[1][0] - coordinates[2][0]) ** 2) + ((coordinates[1][1] - coordinates[2][1]) ** 2))
    area = width * height
    return area, width



def detect_aruco(image):
   
    aruco_area_threshold = 1500

    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    size_of_aruco_m = 0.15

    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
 
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    param_markers = cv2.aruco.DetectorParameters()
    corners,id,rejected = cv2.aruco.detectMarkers(gray,marker_dict,parameters=param_markers)
   
    if len(id)>=0:
        for i in range(len(id)):
            area, width = calculate_rectangle_area(corners[i][0])
            if area>aruco_area_threshold:

                ids.append(id[i])
                topleft_x = corners[i][0][0][0]
                topleft_y = corners[i][0][0][1]
                topright_x = corners[i][0][1][0]
                topright_y = corners[i][0][1][1]
                bottomright_x = corners[i][0][2][0]
                bottomright_y = corners[i][0][2][1]
                bottomleft_x = corners[i][0][3][0]
                bottomleft_y = corners[i][0][3][1]
                center_x = (topleft_x+topright_x+bottomleft_x+bottomright_x)/4
                center_y = (topleft_y+topright_y+bottomleft_y+bottomright_y)/4
                center_aruco_list.append([center_x,center_y])
                width_aruco_list.append(width)
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], size_of_aruco_m, cam_mat, dist_mat)   
                print(f'id , rvec : {id[i],rvec[0][0][2]}') 
                r, _ = cv2.Rodrigues(rvec)
                distance = math.sqrt((tvec[0][0][0]**2)+(tvec[0][0][1]**2)+(tvec[0][0][2]**2))
                distance_from_rgb_list.append(distance)
                rotation = R.from_matrix(r)
                euler_angles = rotation.as_euler('zyx', degrees=True) 
                roll, pitch, yaw = euler_angles
                angle_aruco_list.append([np.radians(roll),np.radians(pitch),np.radians(yaw)])
                center = (int(center_x),int(center_y))
                cv2.circle(image,center ,2, (0, 255, 0), 2)
                cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, 0.3)
                cv2.line(image,(int(topleft_x),int(topleft_y)),(int(topright_x),int(topright_y)),(0,255,0),2)
                cv2.line(image,(int(topright_x),int(topright_y)),(int(bottomright_x),int(bottomright_y)),(0,255,0),2)
                cv2.line(image,(int(bottomright_x),int(bottomright_y)),(int(bottomleft_x),int(bottomleft_y)),(0,255,0),2)
                cv2.line(image,(int(bottomleft_x),int(bottomleft_y)),(int(topleft_x),int(topleft_y)),(0,255,0),2)
            
            
            
            
    else:
        print("No id found")

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids

class aruco_tf(Node):
    
    def __init__(self):

        super().__init__('aruco_tf_publisher')                                          # registering node


        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                     # depth image variable (from depthimagecb())
       

    def depthimagecb(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)

    def colorimagecb(self, data):
       
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


    def process_image(self):
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
        
    
        aruco_center, distance_from_rgb, angle_aruco, width_aruco, ids= detect_aruco(self.cv_image)
        
        

        for i in range(0,len(ids)):
            
            print(f'id : {ids[i]}')
            print(f'angle : {angle_aruco[i][1]}')
            roll = 1.57
            pitch = 0
            yaw = angle_aruco[i][1]+1.57
            if angle_aruco[i][1]<-1:
                pitch -= 0.25
            elif angle_aruco[i][1]>0 and angle_aruco[i][1]<1:
                roll-=0.29
            print(f'roll,pitch,yaw ; {roll},{pitch},{yaw}')
            rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
            q= rotation.as_quat()
            print(f'quaternion : {q}')
            cX , cY = aruco_center[i]

            print(f'cx : {cX} , cy : {cY}')
            depth = self.depth_image[int(cY),int(cX)]
            depth = depth/1000
            x = depth* (sizeCamX - cX - centerCamX) / focalX
            y = depth * (sizeCamY - cY - centerCamY) / focalY
            z = depth
            print(f'distance from rgb : {distance_from_rgb[i]}')
            print(f'depth : {depth}')
            print(f'x,y,z : {x},{y},{z}')

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'camera_link'                                         
            t.child_frame_id = f'cam_{ids[i][0]}'
            t.transform.translation.x = z
            t.transform.translation.y = x                                        
            t.transform.translation.z = y
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.br.sendTransform(t)

            f2 = f'cam_{ids[i][0]}'
            f3 = 'base_link'
            try:
                ts = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())       
                self.get_logger().info(f'Successfully received data!')
                t3 = TransformStamped()
                t3.header.stamp = self.get_clock().now().to_msg()
                t3.header.frame_id = 'base_link'                                         
                t3.child_frame_id = f'obj_{ids[i][0]}'
                t3.transform.translation.x = ts.transform.translation.x
                t3.transform.translation.y = ts.transform.translation.y                                      
                t3.transform.translation.z = ts.transform.translation.z
                t3.transform.rotation.x = ts.transform.rotation.x
                t3.transform.rotation.y = ts.transform.rotation.y
                t3.transform.rotation.z = ts.transform.rotation.z
                t3.transform.rotation.w = ts.transform.rotation.w
                self.br.sendTransform(t3)
                self.get_logger().info(f'Translation X:  {ts.transform.translation.x}')
                self.get_logger().info(f'Translation Y:  {ts.transform.translation.y}')
                self.get_logger().info(f'Translation Z:  {ts.transform.translation.z}')
                self.get_logger().info(f'Rotation X:  {ts.transform.rotation.x}')                                
                self.get_logger().info(f'Rotation Y:  {ts.transform.rotation.y}')
                self.get_logger().info(f'Rotation Z:  {ts.transform.rotation.z}')
                self.get_logger().info(f'Rotation W:  {ts.transform.rotation.w}')


            except TransformException as e:
                self.get_logger().info(f'Could not transform {f3} to {f2}: {e}')
                return
            
        cv2.imshow('image', self.cv_image)
        cv2.waitKey(1)
    




       
def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init()                                                     # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node
    
    node1 = Node("task1b")
    
    callback_group = ReentrantCallbackGroup()

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends
    
    node1.declare_parameter("pick_1", [0.35, 0.1, 0.68])
    node1.declare_parameter("drop", [-0.37, 0.12, 0.397])
    node1.declare_parameter("pick_2", [0.194, -0.43, 0.701])
    node1.declare_parameter("quat_xyzw", [0.0, 0.0, 0.0, 0.1])
    node1.declare_parameter("cartesian", False)

    moveit2 = MoveIt2(
        node=node1,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node1(node1)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    pick_1 = node1.get_parameter("pick_1").get_parameter_value().double_array_value
    drop = node1.get_parameter("drop").get_parameter_value().double_array_value
    pick_2 = node1.get_parameter("pick_2").get_parameter_value().double_array_value
    quat_xyzw = node1.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node1.get_parameter("cartesian").get_parameter_value().bool_value
    
    # Move to pose
    node1.get_logger().info(
    f"Moving to {{position: {list(pick_1)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    moveit2.move_to_pose(position=pick_1, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()
    time.sleep(5)
    
    node1.get_logger().info(
    f"Moving to {{position: {list(drop)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    moveit2.move_to_pose(position=drop, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()
    time.sleep(5)
    
    # Move to pose
    node1.get_logger().info(
    f"Moving to {{position: {list(pick_2)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    moveit2.move_to_pose(position=pick_2, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()
    time.sleep(5)

    node1.get_logger().info(
    f"Moving to {{position: {list(drop)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    moveit2.move_to_pose(position=drop, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()
    time.sleep(5)
    
    gripper_control = self.create_client(AttachLink, '/GripperMagnetON')

    while not gripper_control.wait_for_service(timeout_sec=1.0):
       self.get_logger().info('EEF service not available, waiting again...')

    req = AttachLink.Request()
    req.model1_name =  'box1'      
    req.link1_name  = 'link'       
    req.model2_name =  'ur5'       
    req.link2_name  = 'wrist_3_link'  

    gripper_control.call_async(req)
    
    gripper_control = self.create_client(DetachLink, '/GripperMagnetON')

    while not gripper_control.wait_for_service(timeout_sec=1.0):
       self.get_logger().info('EEF service not available, waiting again...')

    req = DetachLink.Request()
    req.model1_name =  'box1'      
    req.link1_name  = 'link'       
    req.model2_name =  'ur5'       
    req.link2_name  = 'wrist_3_link'  

    gripper_control.call_async(req)

    gripper_control = self.create_client(AttachLink, '/GripperMagnetON')

    while not gripper_control.wait_for_service(timeout_sec=1.0):
       self.get_logger().info('EEF service not available, waiting again...')

    req = AttachLink.Request()
    req.model1_name =  'box3'      
    req.link1_name  = 'link'       
    req.model2_name =  'ur5'       
    req.link2_name  = 'wrist_3_link'  

    gripper_control.call_async(req)
    
    gripper_control = self.create_client(DetachLink, '/GripperMagnetON')

    while not gripper_control.wait_for_service(timeout_sec=1.0):
       self.get_logger().info('EEF service not available, waiting again...')

    req = DetachLink.Request()
    req.model1_name =  'box3'      
    req.link1_name  = 'link'       
    req.model2_name =  'ur5'       
    req.link2_name  = 'wrist_3_link'  

    gripper_control.call_async(req)

    gripper_control = self.create_client(AttachLink, '/GripperMagnetON')

    while not gripper_control.wait_for_service(timeout_sec=1.0):
       self.get_logger().info('EEF service not available, waiting again...')

    req = AttachLink.Request()
    req.model1_name =  'box49'      
    req.link1_name  = 'link'       
    req.model2_name =  'ur5'       
    req.link2_name  = 'wrist_3_link'  

    gripper_control.call_async(req)
    
    gripper_control = self.create_client(DetachLink, '/GripperMagnetON')

    while not gripper_control.wait_for_service(timeout_sec=1.0):
       self.get_logger().info('EEF service not available, waiting again...')

    req = DetachLink.Request()
    req.model1_name =  'box49'      
    req.link1_name  = 'link'       
    req.model2_name =  'ur5'       
    req.link2_name  = 'wrist_3_link'  

    gripper_control.call_async(req)
    

    rclpy.shutdown()                                                # shutdown process
    exit(0)

if __name__ == '__main__':
    main()