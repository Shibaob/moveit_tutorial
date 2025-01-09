#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose

class MoveItIkDemo:
    def __init__(self):
 
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('panda_arm')
                
        # 获取终端link的名称，这个在setup assistant中设置过了
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'panda_link0'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)
       
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        #参考坐标系，前面设置了
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
        #末端位置   
        target_pose.pose.position.x = 0.198694302663
        target_pose.pose.position.y = -0.0723893543524
        target_pose.pose.position.z = 0.341758780532
        #末端姿态，四元数
        target_pose.pose.orientation.x = -0.333017223438
        target_pose.pose.orientation.y = 0.942920732724
        target_pose.pose.orientation.z = -7.71712218442e-05
        target_pose.pose.orientation.w = 0.000121399332436
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径，返回虚影的效果
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(1)  #执行完成后休息1s

        target_pose2 = PoseStamped()
        #参考坐标系，前面设置了
        target_pose2.header.frame_id = reference_frame
        target_pose2.header.stamp = rospy.Time.now()
        #末端位置   
        target_pose2.pose.position.x = 0.516160769806
        target_pose2.pose.position.y = 0.363391472307
        target_pose2.pose.position.z = 0.50578621851
        #末端姿态，四元数
        target_pose2.pose.orientation.x =-0.332982257701
        target_pose2.pose.orientation.y = 0.942933076679
        target_pose2.pose.orientation.z = -0.000169565872356
        target_pose2.pose.orientation.w = 1.44284963199e-05

        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose2, end_effector_link)
        
        # 规划运动路径，返回虚影的效果
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(1)  #执行完成后休息1s

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    MoveItIkDemo()
