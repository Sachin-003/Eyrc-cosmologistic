if angle_aruco[i][1]<-1:
                pitch -= 0.25
            elif angle_aruco[i][1]>0 and angle_aruco[i][1]<1:
                roll-=0.29



                roll = 1.57
            pitch = 0
            yaw = angle_aruco[i][1]+1.57


navigator = BasicNavigator()

                initial_pose = PoseStamped()
                initial_pose.header.frame_id = 'map'
                initial_pose.header.stamp = navigator.get_clock().now().to_msg()
                initial_pose.pose.position.x = self.robot_pose[0]
                initial_pose.pose.position.y = self.robot_pose[1]
                initial_pose.pose.orientation.z = self.robot_orientation_z
                initial_pose.pose.orientation.w = self.robot_orientation_w
                navigator.setInitialPose(initial_pose)

                navigator.waitUntilNav2Active()

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = 0.5
                goal_pose.pose.position.y = -2.0
                goal_pose.pose.orientation.z = 1.0
                goal_pose.pose.orientation.w = 0.0

                navigator.goToPose(goal_pose)
                self.undocking = True
                
                i = 0
                while not navigator.isTaskComplete():
                    i = i + 1
                    feedback = navigator.getFeedback()
                    if feedback and i % 5 == 0:
                        print('Estimated time of arrival: ' + '{0:.0f}'.format(
                            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                            + ' seconds.')
                        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                            navigator.cancelTask()


                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    print('Goal succeeded!')
                elif result == TaskResult.CANCELED:
                    print('Goal was canceled!')
                elif result == TaskResult.FAILED:
                    print('Goal failed!')
                else:
                    print(result)

                navigator.lifecycleShutdown()

                v_msg.linear.x = 0.0
                v_msg.angular.z = 0.0
                self.publisher_cmd_vel.publish(v_msg)


if (self.dist_from_rack<0.5):
                v_msg.linear.x = 0.08
                v_msg.angular.z = 0.0
                self.publisher_cmd_vel.publish(v_msg)
                

            if (self.dist_from_rack>=0.5):
                navigator = BasicNavigator()

                initial_pose = PoseStamped()
                initial_pose.header.frame_id = 'map'
                initial_pose.header.stamp = navigator.get_clock().now().to_msg()
                initial_pose.pose.position.x = self.robot_pose[0]
                initial_pose.pose.position.y = self.robot_pose[1]
                initial_pose.pose.orientation.z = self.robot_orientation_z
                initial_pose.pose.orientation.w = self.robot_orientation_w
                navigator.setInitialPose(initial_pose)

                navigator.waitUntilNav2Active()

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = 0.5
                goal_pose.pose.position.y = -2.455
                goal_pose.pose.orientation.z = 1.0
                goal_pose.pose.orientation.w = 0.0

                navigator.goToPose(goal_pose)
                self.undocking = True
                
                i = 0
                while not navigator.isTaskComplete():
                    i = i + 1
                    feedback = navigator.getFeedback()
                    if feedback and i % 5 == 0:
                        print('Estimated time of arrival: ' + '{0:.0f}'.format(
                            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                            + ' seconds.')
                        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                            navigator.cancelTask()


                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    print('Goal succeeded!')
                elif result == TaskResult.CANCELED:
                    print('Goal was canceled!')
                elif result == TaskResult.FAILED:
                    print('Goal failed!')
                else:
                    print(result)

                navigator.lifecycleShutdown()

                
                




            pass



result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    print('Goal succeeded!')
                elif result == TaskResult.CANCELED:
                    print('Goal was canceled!')
                elif result == TaskResult.FAILED:
                    print('Goal failed!')
                else:
                    print(result)

                
                self.undocking = True
                
        if self.undocking :
            while not self.link_detach_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Link detacher service not available, waiting again...')

            req = DetachLink.Request()
            req.model1_name =  'ebot'     
            req.link1_name  = 'ebot_base_link'       
            req.model2_name =  'rack1'       
            req.link2_name  = 'link'  

            self.link_detach_cli.call_async(req)

            

            goal_pose2 = PoseStamped()
            goal_pose2.header.frame_id = 'map'
            goal_pose2.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose2.pose.position.x = 0.0
            goal_pose2.pose.position.y = 0.0
            goal_pose2.pose.orientation.z = 0.0
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
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print(result)
            self.navigator.lifecycleShutdown()

            box1_coordinates = Float64MultiArray()
            box1_coordinates.data = [xyz_baselink[0][0],xyz_baselink[0][1],xyz_baselink[0][2]]
            self.publisher1.publish(box1_coordinates)
            self.get_logger().info(f'Publishing: {box1_coordinates.data}')


box2_coordinates = Float64MultiArray()
            box2_coordinates.data = xyz_baselink[1]
            self.publisher2.publish(box2_coordinates)
            self.get_logger().info(f'Publishing: {box2_coordinates.data}')

            box3_coordinates = Float64MultiArray()
            box3_coordinates.data = xyz_baselink[2]
            self.publisher3.publish(box3_coordinates)
            self.get_logger().info(f'Publishing: {box3_coordinates.data}')

            id_data = Float64MultiArray()
            id_data.data = [ids[0][0] , ids[1][0] , ids[2][0]]
            self.id_publisher.publish(id_data)
            self.get_logger().info(f'Publishing: {id_data.data}')



__twist_msg = TwistStamped()
        dist = math.sqrt((self.box2_coordinates[0]**2)+(self.box2_coordinates[1]**2)+(self.box2_coordinates[2]**2))
        current_time = self.get_clock().now()
        
        __twist_msg.header.frame_id = ur5.base_link_name()
        __twist_msg.header.stamp = current_time.to_msg()
        __twist_msg.twist.linear.x = self.box2_coordinates[0]
        __twist_msg.twist.linear.y = self.box2_coordinates[1]
        __twist_msg.twist.linear.z = self.box2_coordinates[2]-0.4
        __twist_msg.twist.angular.x = 0.0
        __twist_msg.twist.angular.y = 0.0
        __twist_msg.twist.angular.z = 0.0
        self.__twist_pub.publish(__twist_msg)


    def move_arm(self):

        node = Node("ex_joint_goal")

        home_pose = [0.0 , -2.3911 , 2.40855 , -3.14159 , -1.58825 , 3.14159]
        inter_pose = [1.57, -1.57, 0.0, -1.57, 0.0, 1.57]
        joint_pose = [0,0,0,0,0,0]
        drop_pose = [-0.0174533, -1.90241 , -1.32645 , 0.10472 , 1.71042 ,3.14159]
        f2 = 'tool0'
        f3 = 'base_link'
        ts = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
        if self.angle_list[0]>1 and self.angle_list[0]<3 :
            joint_pose = [-0.174533 , -1.91986 , 1.74533 , -2.96706,-1.39626, 3.14159 ]

        if self.angle_list[0]<1:
            joint_pose = [-1.39626 , -1.62316 , 1.46608 , -2.98451 , -1.5708 , 3.15905 ] 

        if self.angle_list[0]>3:
            joint_pose = [0.925025 , -1.23918 , 1.06465 , -2.98451 , 5.48033 , 3.15905]



        moveit2 = MoveIt2(
            node=node,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM
            )
        
        node.get_logger().info(f"Moving to {{joint_positions: {list(joint_pose)}}}")
        moveit2.move_to_configuration(joint_pose)
        node.get_logger().info(f"Moving to {{joint_positions: {list(home_pose)}}}")
        moveit2.move_to_configuration(home_pose)
        node.get_logger().info(f"Moving to {{joint_positions: {list(inter_pose)}}}")
        moveit2.move_to_configuration(inter_pose)
        node.get_logger().info(f"Moving to {{joint_positions: {list(drop_pose)}}}")
        moveit2.move_to_configuration(drop_pose)
        node.get_logger().info(f"Moving to {{joint_positions: {list(inter_pose)}}}")
        moveit2.move_to_configuration(inter_pose)
        node.get_logger().info(f"Moving to {{joint_positions: {list(home_pose)}}}")
        moveit2.move_to_configuration(home_pose)

        if self.angle_list[1]>1 and self.angle_list[1]<3 :
            joint_pose = [-0.174533 , -1.91986 , 1.74533 , -2.96706,-1.39626, 3.14159 ]

        if self.angle_list[1]<1:
            joint_pose = [-1.39626 , -1.62316 , 1.46608 , -2.98451 , -1.5708 , 3.15905 ] 

        if self.angle_list[1]>3:
            joint_pose = [0.925025 , -1.23918 , 1.06465 , -2.98451 , 5.48033 , 3.15905]

        node.get_logger().info(f"Moving to {{joint_positions: {list(joint_pose)}}}")
        moveit2.move_to_configuration(joint_pose)
        node.get_logger().info(f"Moving to {{joint_positions: {list(home_pose)}}}")
        moveit2.move_to_configuration(home_pose)
        node.get_logger().info(f"Moving to {{joint_positions: {list(inter_pose)}}}")
        moveit2.move_to_configuration(inter_pose)
        node.get_logger().info(f"Moving to {{joint_positions: {list(drop_pose)}}}")
        moveit2.move_to_configuration(drop_pose)
        node.get_logger().info(f"Moving to {{joint_positions: {list(inter_pose)}}}")
        moveit2.move_to_configuration(inter_pose)
        node.get_logger().info(f"Moving to {{joint_positions: {list(home_pose)}}}")
        moveit2.move_to_configuration(home_pose)

        if self.angle_list[2]>1 and self.angle_list[2]<3 :
            joint_pose = [-0.174533 , -1.91986 , 1.74533 , -2.96706,-1.39626, 3.14159 ]

        if self.angle_list[2]<1:
            joint_pose = [-1.39626 , -1.62316 , 1.46608 , -2.98451 , -1.5708 , 3.15905 ] 

        if self.angle_list[2]>3:
            joint_pose = [0.925025 , -1.23918 , 1.06465 , -2.98451 , 5.48033 , 3.15905]

        node.get_logger().info(f"Moving to {{joint_positions: {list(joint_pose)}}}")
        moveit2.move_to_configuration(joint_pose)
        node.get_logger().info(f"Moving to {{joint_positions: {list(home_pose)}}}")
        moveit2.move_to_configuration(home_pose)
        node.get_logger().info(f"Moving to {{joint_positions: {list(inter_pose)}}}")
        moveit2.move_to_configuration(inter_pose)
        node.get_logger().info(f"Moving to {{joint_positions: {list(drop_pose)}}}")
        moveit2.move_to_configuration(drop_pose)
        node.get_logger().info(f"Moving to {{joint_positions: {list(inter_pose)}}}")
        moveit2.move_to_configuration(inter_pose)
        node.get_logger().info(f"Moving to {{joint_positions: {list(home_pose)}}}")
        moveit2.move_to_configuration(home_pose)
        
        moveit2.wait_until_executed()



        try:
    # Code that executes the 'move_action'
except Exception as e:
    node.get_logger().error(f"Error executing 'move_action': {str(e)}")



if (self.angle_list[0]>1 and self.angle_list[0]<3):
            self.in_correct_joints = True
        
        if (self.angle_list[0]<1) and (not self.in_correct_joints):
            try:
                initial_pose[0] = -1.57
                self.get_logger().info(f"Moving to {{joint_positions: {list(initial_pose)}}}")
                moveit2.move_to_configuration(initial_pose)
            
                self.in_correct_joints = True
            except Exception as e:
                self.get_logger().error(f"Error executing 'move_action': {str(e)}")
                

           

        if self.angle_list[0]>3 and (not self.in_correct_joints):
            try:
                initial_pose[0] = 1.57
                self.get_logger().info(f"Moving to {{joint_positions: {list(initial_pose)}}}")
                moveit2.move_to_configuration(initial_pose)
            
                self.in_correct_joints = True
            except Exception as e:
                self.get_logger().error(f"Error executing 'move_action': {str(e)}")
           
        






        __twist_msg = TwistStamped()
        current_time = self.get_clock().now()
        __twist_msg.header.frame_id = ur5.base_link_name()
        __twist_msg.header.stamp = current_time.to_msg()

        if (abs(dist_z)<0.1):
            self.z_align = True
        
        if (abs(dist_y)<0.1):
            self.y_align = True

        if (abs(dist_x)<0.1):
            self.x_align = True

        if((not self.z_align)and self.in_correct_joints):
            __twist_msg.twist.linear.x = 0.0
            __twist_msg.twist.linear.y = 0.0
            __twist_msg.twist.linear.z = vz
            __twist_msg.twist.angular.x = 0.0
            __twist_msg.twist.angular.y = 0.0
            __twist_msg.twist.angular.z = 0.0
            self.__twist_pub.publish(__twist_msg)

        if (self.angle_list[0]>1 and self.angle_list[0]<3 and (not self.y_align) and (self.z_align)and (self.in_correct_joints)):
            __twist_msg.twist.linear.x = 0.0
            __twist_msg.twist.linear.y = vy
            __twist_msg.twist.linear.z = 0.0
            __twist_msg.twist.angular.x = 0.0
            __twist_msg.twist.angular.y = 0.0
            __twist_msg.twist.angular.z = 0.0
            self.__twist_pub.publish(__twist_msg)

        if ((not (self.angle_list[0]>1 and self.angle_list[0]<3)) and (self.z_align) and (not self.x_align)and (self.in_correct_joints)):
            __twist_msg.twist.linear.x = vx
            __twist_msg.twist.linear.y = 0.0
            __twist_msg.twist.linear.z = 0.0
            __twist_msg.twist.angular.x = 0.0
            __twist_msg.twist.angular.y = 0.0
            __twist_msg.twist.angular.z = 0.0
            self.__twist_pub.publish(__twist_msg)




            while (not self.in_correct_joints):

            initial_pose = [0.0 , -2.3911 , 2.40855 , -3.14159 , -1.58825 , 3.14159]
            print("haa bhai ruk ja")
            node = Node("ex_joint_goal")
            moveit2 = MoveIt2(
                node = node,
                joint_names=ur5.joint_names(),
                base_link_name=ur5.base_link_name(),
                end_effector_name=ur5.end_effector_name(),
                group_name=ur5.MOVE_GROUP_ARM,
                callback_group=self.callback_group
                )
            self.get_logger().info(f"Moving to {{joint_positions: {list(initial_pose)}}}")
            moveit2.move_to_configuration(initial_pose)
            self.in_correct_joints = True
            print(self.in_correct_joints)
            time.sleep(2)

        
        
        while (abs(dist_z)>0.01):
            t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
            tool0_x = t.transform.translation.x
            tool0_y = t.transform.translation.y
            tool0_z = t.transform.translation.z
            __twist_msg = TwistStamped()
            current_time = self.get_clock().now()
            __twist_msg.header.frame_id = ur5.base_link_name()
            __twist_msg.header.stamp = current_time.to_msg()
            __twist_msg.twist.linear.x = 0.0
            __twist_msg.twist.linear.y = 0.0
            __twist_msg.twist.linear.z = vz
            __twist_msg.twist.angular.x = 0.0
            __twist_msg.twist.angular.y = 0.0
            __twist_msg.twist.angular.z = 0.0
            self.__twist_pub.publish(__twist_msg)
            dist_z = tool0_z - self.box1_coordinates[2]
            print(f"dist_z :{dist_z}") 
            time.sleep(0.2)





for i in range(0,len(self.id_list)):

            

            vx = 0.2
            vy = 0.2
            vz = 0.2

            dist_x = boxes_coordinates[i][0] - tool0_x
            dist_y = boxes_coordinates[i][1] - tool0_y
            dist_z = boxes_coordinates[i][2] - tool0_z

            if dist_x<0: 
                vx = -0.2

            if dist_y<0: 
                vy = -0.2

            if dist_z<0: 
                vz = -0.2

            distance = math.sqrt((dist_x**2)+(dist_y**2)+(dist_z**2))

            self.in_correct_joints = False
            initial_pose = [0.0 , -2.3911 , 2.40855 , -3.14159 , -1.58825 , 3.14159]

            if self.angle_list[i]<1 :
                initial_pose[0] = -1.57

            if self.angle_list[i]>3 :
                initial_pose[0] = 1.57

            if self.angle_list[i]>1 and self.angle_list[i]<3 :
                self.in_correct_joints = True

            while (not self.in_correct_joints):

                
                mv_obj = MoveIt2(
                    node = Node(f'nn{i}'),
                    joint_names=ur5.joint_names(),
                    base_link_name=ur5.base_link_name(),
                    end_effector_name=ur5.end_effector_name(),
                    group_name=ur5.MOVE_GROUP_ARM,
                    callback_group=self.callback_group
                )

                self.get_logger().info(f"Moving to {{joint_positions: {list(initial_pose)}}}")
                mv_obj.move_to_configuration(initial_pose)
                self.in_correct_joints = True
                print(self.in_correct_joints)
                time.sleep(4)
                

            while (abs(dist_z)>0.01):
                t = self.tf_buffer.lookup_transform( f3, f2, rclpy.time.Time())
                tool0_x = t.transform.translation.x
                tool0_y = t.transform.translation.y
                tool0_z = t.transform.translation.z
                __twist_msg = TwistStamped()
                current_time = self.get_clock().now()
                __twist_msg.header.frame_id = ur5.base_link_name()
                __twist_msg.header.stamp = current_time.to_msg()
                __twist_msg.twist.linear.x = 0.0
                __twist_msg.twist.linear.y = 0.0
                __twist_msg.twist.linear.z = vz
                __twist_msg.twist.angular.x = 0.0
                __twist_msg.twist.angular.y = 0.0
                __twist_msg.twist.angular.z = 0.0
                self.__twist_pub.publish(__twist_msg)
                dist_z = tool0_z - boxes_coordinates[i][2]
                print(f"dist_z :{dist_z}") 
                time.sleep(0.2)






                goal_pose2 = PoseStamped()
        goal_pose2.header.frame_id = 'map'
        goal_pose2.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose2.pose.position.x = 0.5
        goal_pose2.pose.position.y = -2.455
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
        self.cli.call_async(self.req)