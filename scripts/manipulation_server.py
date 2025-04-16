#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''这段代码实现了一个机械臂的头部转动动作：移动到目标姿态、转动头部、再次回到目标姿态'''

import numpy as np
import rospy
import cashmerebot.msg
import actionlib
from plot_marker import *
from transform_tools import *
from geometry_msgs.msg import Pose
import sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from copy import deepcopy


# params
class manipulation_action(object):
    # 初始化动作服务器的反馈和结果消息类型
    _feedback = cashmerebot.msg.manipulationFeedback()
    _result = cashmerebot.msg.manipulationResult()

    # 初始化动作服务器
    def __init__(self):
        self._action_name = 'manipulation'  # 设置动作服务器的名称
        # 创建了一个动作服务器实例，指定了动作名称、消息类型和执行回调函数
        self._as = actionlib.SimpleActionServer(self._action_name, cashmerebot.msg.manipulationAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # 初始化MoveIt! Commander所使用的ROS C++节点，使用从命令行传递给脚本的参数。
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = MoveGroupCommander('manipulator')

        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)

        # 设置目标位置所使用的参考坐标系
        self.arm.set_pose_reference_frame('base_link')

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)

        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()

        # 设置机械臂的初始位姿和执行前的位姿
        self.home_pose = np.deg2rad(np.array([0, -150, 60, 90, 90, 0])).tolist()
        self.pre_exec_pose = np.deg2rad(np.array([0, -95, 48, 90, 90, 0])).tolist()

        # 设置转动头部的阈值
        self.turn_head_z = 0.45

        # 将机械臂移动到初始位姿
        self.arm.set_joint_value_target(self.home_pose)
        # 控制机械臂完成运动
        self.arm.go()
        rospy.sleep(1)

    # 执行回调函数，当动作服务器接收到目标时被调用
    def execute_cb(self, goal):  # goal.path: list of poses in terms of trans+ypr
        success = False
        # goal.path 是一个包含路径点的列表，每个路径点可能包含位置和姿态信息
        # -1 表示自动计算该维度的大小，以确保数组中的元素总数保持不变
        # .reshape将 goal.path 转换为一个NumPy数组，并重塑为每行6列的形式，.T将NumPy数组转置
        path_np = np.array(goal.path).reshape((-1, 6)).T
        # 获取数组 path_np 的列数，并将这个值赋给变量 path_pose_num
        path_pose_num = path_np.shape[1]

        rospy.loginfo('%s: obtained path, executing ...' % (self._action_name))

        # add poses to moveit planner

        # 初始化路点列表
        # waypoints = []

        # 获取当前位姿数据作为机械臂运动的起始位姿
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
        # 将初始位姿加入路点列表
        # waypoints.append(start_pose)

        # go to pre_exe pose
        # self.arm.set_joint_value_target(self.pre_exec_pose)
        # 控制机械臂完成运动
        # self.arm.go()
        # rospy.sleep(1)
        self.run_joint_target_one_by_one(self.pre_exec_pose)

        ## 初始化变量
        head_turned = False  # 用于跟踪是否已经执行了头部转动动作

        ## 遍历路径点
        for n in range(path_pose_num):
            pose = path_np[:, n]

            ## 转换路径点:
            # 将路径点的位置和姿态（以欧拉角YPR表示）转换为旋转矩阵
            m = transform_trans_ypr_to_matrix(pose)
            # 通过矩阵乘法将工具的局部坐标系变换到全局坐标系
            tool_m = m @ transform_trans_ypr_to_matrix(np.array([0, 0, 0, np.pi / 2, 0, np.pi / 2])) @ \
                np.linalg.pinv(transform_trans_ypr_to_matrix(np.array([0, 0.06, 0.08, 0, 0, 0])))
            # 将旋转矩阵转换为四元数,将转换得到的四元数数组调整为一维数组
            tool_pose_quat = transform_matrix_to_trans_quat(tool_m).reshape((-1,))

            # 如果末端执行器的高度低于某个阈值，并且当前是偶数编号的路径点，且头部尚未转动，则执行头部转动动作
            # if (tool_pose_quat[2] < self.turn_head_z) and (n % 2 == 0) and (not head_turned):
            #     self.turn_head(tool_pose_quat)  # 调用 turn_head 函数来转动头部
            #     head_turned = True  # 表示头部已经转动

            ## 设置目标姿态
            #  创建一个 Pose 对象 tar_pose
            #  将 tool_pose_quat 数组中的值分别赋给 tar_pose 的位置和姿态属性。
            tar_pose = Pose()
            tar_pose.position.x = tool_pose_quat[0]
            tar_pose.position.y = tool_pose_quat[1]
            tar_pose.position.z = tool_pose_quat[2]
            tar_pose.orientation.x = tool_pose_quat[3]
            tar_pose.orientation.y = tool_pose_quat[4]
            tar_pose.orientation.z = tool_pose_quat[5]
            tar_pose.orientation.w = tool_pose_quat[6]

            ## 移动机械臂
            self.arm.set_pose_target(tar_pose)  # 设置机械臂的目标姿态
            # self.arm.plan()
            # self.arm.execute()
            self.arm.go()  # 命令机械臂移动到目标姿态
            rospy.sleep(1)

        ##  返回到初始姿态
        self.run_joint_target_one_by_one(self.home_pose)

        ## 设置成功标志并返回结果
        success = True
        # 如果成功标志为真，则记录信息并设置动作服务器为成功状态
        if success == True:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

    ## 逐步地将机械臂的每个关节移动到指定的目标位置
    def run_joint_target_one_by_one(self, joint_target):
        # 取机械臂当前所有关节的角度值，并将其存储在 joint_target_step 列表中
        joint_target_step = self.arm.get_current_joint_values()

        # 遍历所有关节
        for n in range(6):
            # 在每次循环中，将 joint_target 中对应关节的目标角度值赋给 joint_target_step 中的相应位置
            joint_target_step[n] = joint_target[n]
            # 将机械臂的关节目标设置为 joint_target_step 中的角度值
            self.arm.set_joint_value_target(joint_target_step)
            # 控制机械臂完成运动
            self.arm.go()
            rospy.sleep(0.1)

    def turn_head(self, tool_pose_quat):
        # go to target pose
        #  创建一个 Pose 对象 tar_pose
        #  将 tool_pose_quat 数组中的值分别赋给 tar_pose 的位置和姿态属性。
        tar_pose = Pose()
        tar_pose.position.x = tool_pose_quat[0]
        tar_pose.position.y = tool_pose_quat[1]
        tar_pose.position.z = tool_pose_quat[2]
        tar_pose.orientation.x = tool_pose_quat[3]
        tar_pose.orientation.y = tool_pose_quat[4]
        tar_pose.orientation.z = tool_pose_quat[5]
        tar_pose.orientation.w = tool_pose_quat[6]

        self.arm.set_pose_target(tar_pose)  # 设置机械臂的目标姿态
        # self.arm.plan()
        # self.arm.execute()
        self.arm.go()  # 机械臂移动到该目标姿态
        rospy.sleep(1)

        ## 转动头部
        # 获取机械臂当前的关节值
        joint_target_step = self.arm.get_current_joint_values()
        # 对 joint_target_step 中的特定关节值进行调整，以实现头部转动
        joint_target_step[1] = joint_target_step[1] - np.deg2rad(10)
        # joint_target_step[2] = joint_target_step[2] + np.deg2rad(10)

        ## 处理角度翻转
        # 如果关节3、4、5当前角度小于0，加上 np.deg2rad(180)，否则减去该值，确保角度在正确的范围内
        if joint_target_step[3] < 0:
            joint_target_step[3] = joint_target_step[3] + np.deg2rad(180)
        else:
            joint_target_step[3] = joint_target_step[3] - np.deg2rad(180)
        if joint_target_step[4] < 0:
            joint_target_step[4] = joint_target_step[4] + np.deg2rad(180)
        else:
            joint_target_step[4] = joint_target_step[4] - np.deg2rad(180)
        if joint_target_step[5] < 0:
            joint_target_step[5] = joint_target_step[5] + np.deg2rad(180)
        else:
            joint_target_step[5] = joint_target_step[5] - np.deg2rad(180)
        # 逐步将每个关节移动到新的目标位置
        self.run_joint_target_one_by_one(joint_target_step)

        # 再次移动到目标姿态
        tar_pose = Pose()
        tar_pose.position.x = tool_pose_quat[0]
        tar_pose.position.y = tool_pose_quat[1]
        tar_pose.position.z = tool_pose_quat[2]
        tar_pose.orientation.x = tool_pose_quat[3]
        tar_pose.orientation.y = tool_pose_quat[4]
        tar_pose.orientation.z = tool_pose_quat[5]
        tar_pose.orientation.w = tool_pose_quat[6]

        self.arm.set_pose_target(tar_pose)
        # self.arm.plan()
        # self.arm.execute()
        self.arm.go()
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        rospy.init_node('manipulation_server_node')
        server = manipulation_action()
        rospy.spin()
    except rospy.ROSInterruptException:
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)