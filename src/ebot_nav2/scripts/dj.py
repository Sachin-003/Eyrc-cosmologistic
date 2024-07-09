goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 1.8
    goal_pose1.pose.position.y = 1.5
    goal_pose1.pose.orientation.w = 0.71
    goal_pose1.pose.orientation.z = 0.0
    goal_poses.append(goal_pose1)

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 2.0
    goal_pose2.pose.position.y = -7.0
    goal_pose2.pose.orientation.w = 0.71
    goal_pose2.pose.orientation.z = 0.0
    goal_poses.append(goal_pose2)

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -3.0
    goal_pose3.pose.position.y = 2.5
    goal_pose3.pose.orientation.w = 0.71
    goal_pose3.pose.orientation.z = 0.0
    goal_poses.append(goal_pose3)

    navigator.goThroughPoses(goal_poses)