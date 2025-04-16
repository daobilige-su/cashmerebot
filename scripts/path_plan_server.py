#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
这段代码实现了一个ROS动作服务器，用于接收路径规划任务，处理点云数据，进行路径规划，并将规划结果可视化和返回
'''

import numpy as np
import rospy
import cashmerebot.msg
import actionlib
from geometry_msgs.msg import Twist            # 通常用于描述机器人的线性和角速度
from sensor_msgs.msg import PointCloud2        # 处理和传递3D点云数据
from std_msgs.msg import Int8MultiArray        # 传递一维数组数据
import cv2                                     # 导入OpenCV库
from cv_bridge import CvBridge, CvBridgeError  # 用于在ROS消息和OpenCV图像格式之间进行转换
import datetime                                # 用于处理日期和时间
import time                                    # 提供各种时间相关的函数
import ros_numpy
from plot_marker import *                      # 用于在ROS RViz中绘制标记或其他可视化元素的自定义模块

# params
class path_plan_action(object):
    # 初始化了动作服务器的反馈（_feedback）和结果（_result）消息对象
    _feedback = cashmerebot.msg.path_planFeedback()
    _result = cashmerebot.msg.path_planResult()

    # 类的构造函数，用于初始化类的实例
    def __init__(self):
        self._action_name = 'path_plan'  # 定义了一个名为path_plan的动作名称
        # 回调函数，当动作服务器接收到一个目标时会被调用
        self._as = actionlib.SimpleActionServer(self._action_name, cashmerebot.msg.path_planAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()  # 手动启动动作服务器
        # params:
        self.pc_roi = np.array([[0.2, 0.8], [-1, 1], [0, 1]])
        # 创建了一个ROS发布者，用于发布MarkerArray消息类型，通常用于在ROS的rviz工具中显示标记
        self.marker_pub = rospy.Publisher("path_plan_markers", MarkerArray, queue_size=5)
        self.surf_dist = 0.05   # 直线移动：定义机器人在水平面上的距离增量
        self.lift_dist = 0.05  # 提升或下降：定义机器人在垂直面上降的距离增量
        self.angle_incr = 5    # 旋转：定义机器人的角度增量
        self.ee_base_y_shift = 0.11  # 用于定义末端执行器相对于基座在y轴方向上的偏移

    # 定义动作服务器的执行回调函数execute_cb，接受表示执行任务目标的参数goal。
    def execute_cb(self, goal): # 0: stand still, 1: forward; 2, backward; 3, left; 4, right;
        success = False  # 用于跟踪路径规划是否成功
        params = goal.params  # 从goal对象中提取包含执行任务信息的参数

        # 记录信息，告知用户动作服务器正在执行任务，并输出目标位置的参数
        rospy.loginfo('%s: Executing, obtained goal location: (%f)' % (self._action_name, goal.params[0]))

        # 使用rospy.wait_for_message等待并接收来自/front_depth_cam/pc2_ur5_base话题的PointCloud2消息，最多等待1秒
        pc2_msg = rospy.wait_for_message('/front_depth_cam/pc2_ur5_base', PointCloud2, timeout=1.0)

        # 将接收到的PointCloud2消息转换为一个包含XYZ坐标的numpy数组，并转置为3行N列的数组（每行代表一个点的XYZ坐标）
        cloud = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg).T

        # 复制点云数组，定义感兴趣区域（ROI）
        cloud_roi = cloud.copy()

        # 根据self.pc_roi定义的X坐标范围，筛选出点云中的点
        cloud_roi = cloud_roi[:, cloud_roi[0, :] > self.pc_roi[0, 0]]
        cloud_roi = cloud_roi[:, cloud_roi[0, :] < self.pc_roi[0, 1]]
        # 根据self.pc_roi定义的Y坐标范围，进一步筛选点云中的点
        cloud_roi = cloud_roi[:, cloud_roi[1, :] > self.pc_roi[1, 0]]
        cloud_roi = cloud_roi[:, cloud_roi[1, :] < self.pc_roi[1, 1]]
        # 根据self.pc_roi定义的Z坐标范围，完成点云的筛选
        cloud_roi = cloud_roi[:, cloud_roi[2, :] > self.pc_roi[2, 0]]
        cloud_roi = cloud_roi[:, cloud_roi[2, :] < self.pc_roi[2, 1]]

        # 复制经过ROI筛选后的点云，用于后续的分割处理
        cloud_seg = cloud_roi.copy()

        # 定义一个中线mid_line，用于路径规划的参考线
        mid_line = np.array([[0.63, 0.63], [-0.9, 0.9], [0.4, 0.4]])

        # 复制cloud_seg数组，用于后续的路径规划
        cloud_ring = cloud_seg.copy()
        # 根据定义的Y坐标范围和偏移量，筛选出特定高度范围内的点，代表末端执行器的移动范围
        cloud_ring = cloud_ring[:, cloud_ring[1, :] > self.ee_base_y_shift-0.05]
        cloud_ring = cloud_ring[:, cloud_ring[1, :] < self.ee_base_y_shift+0.05]

        # 调用plot_pts函数，转置筛选后的点云cloud_ring，使用self.marker_pub发布到path_plan_markers话题，在rviz中可视化
        plot_pts(cloud_ring.T, self.marker_pub, 0, 'ur5_base')
        # 调用plot_traj函数，将mid_line的转置形式作为轨迹线发布，用于在rviz中可视化
        plot_traj(mid_line.T, self.marker_pub, 10, 'ur5_base', 0.02)

        ring_center = mid_line[:, 0].copy() # 从mid_line中提取第一个点作为环的中心
        ring_center[1] = self.ee_base_y_shift  # 将其Y坐标设置为self.ee_base_y_shift
        ring_center = ring_center.reshape((-1,)) # 重塑数组

        # 调用self.plan_single_ring函数进行路径规划，传入筛选后的点云和环中心，得到路径姿态path_pose
        path_pose = self.plan_single_ring(cloud_ring, ring_center)

        # plot_traj(can_pose_ypr_valid[0:3, :].T, self.marker_pub, 2, 'ur5_base')
        # 使用plot_arrows函数可视化路径的方向
        plot_arrows(path_pose.T, self.marker_pub, 100, 'ur5_base')
        # 使用plot_traj函数可视化路径的姿态
        plot_traj(path_pose[0:3, :].T, self.marker_pub, 20, 'ur5_base')

        # 将路径姿态path_pose转换为一维数组，然后转换为列表
        path_list = path_pose.T.reshape((-1,)).tolist()

        # 将路径列表存储在结果对象self._result的path属性中
        self._result.path = path_list

        # 表示路径规划成功
        success = True

        # 如果success为True，则记录成功信息，将结果发送给客户端，完成动作
        if success == True:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

    # 定义 plan_single_ring 函数，接受两个参数：cloud（点云数据）和 center（中心点坐标）
    def plan_single_ring(self, cloud, center):
        cloud_2d = cloud[[0, 2], :].copy()  # 从点云数据中提取X和Z坐标（2D视图），并复制到新的数组 cloud_2d
        center_2d = center[[0, 2]].copy()   # 提取中心点的X和Z坐标，并复制到新的数组 center_2d

        # 使用 np.arctan2 计算每个点相对于中心点的角度，返回给定的Y和X坐标差的反正切值，用于获取角度
        angle = np.arctan2(cloud_2d[1, :]-center_2d[1], -(cloud_2d[0, :]-center_2d[0]))
        angle_deg = np.rad2deg(angle)  # 将角度从弧度转换为度

        # 创建一个从90度到-90度（减去一个角度增量），步长为 -self.angle_incr 的角度列表
        can_deg = range(80, -70-self.angle_incr, -self.angle_incr)
        can_num = len(can_deg)  # 计算可能路径方向的数量
        can_pts_num = np.zeros((1, can_num))  # 初始化一个数组，用于存储每个可能方向上的点的数量

        # 初始化两个6xcan_num 的零矩阵，用于存储每个可能方向上的姿态（位置和方向角）
        can_pose_ypr = np.zeros((6, can_num))
        can_lift_pose_ypr = np.zeros((6, can_num))

        for idx in range(can_num):   # 开始一个循环，用于迭代所有可能的方向
            deg = can_deg[idx]       # 获取当前迭代的角度

            # 确定当前角度与每个点的角度差是否小于半个角度增量，以筛选出当前角度附近的点
            deg_cloud_idx = (abs(angle_deg-deg)<(self.angle_incr/2.0))
            deg_cloud_pts = cloud_2d[:, deg_cloud_idx].copy()   # 根据筛选条件复制当前角度附近的点
            deg_cloud_pts_num = deg_cloud_pts.shape[1]     # 计算当前角度附近的点的数量
            can_pts_num[0, idx] = deg_cloud_pts_num        # 将当前角度的点的数量存储到 can_pts_num 数组

            # 如果当前角度的点的数量少于5，跳过当前迭代
            if deg_cloud_pts_num<5:
                continue

            # 计算当前角度附近点的均值
            deg_cloud_pts_mean = np.mean(deg_cloud_pts, 1)
            # 计算中心点到点云均值的欧几里得距离
            dist_mean = np.sqrt(np.sum(np.power(center_2d - deg_cloud_pts_mean, 2)))
            # 计算机器人末端执行器到点云均值的距离，加上一个表面距离增量
            dist_arm = dist_mean+self.surf_dist
            # 计算机器人末端执行器提升后到点云均值的距离，再加上一个提升距离增量
            dist_arm_lift = dist_mean+self.surf_dist+self.lift_dist

            # 计算当前角度下，机器人末端执行器的位置和方向角（YPR），并重塑为6x1的矩阵
            deg_cloud_pose = np.array([center_2d[0]-dist_arm*np.cos(np.deg2rad(deg)), center[1], center_2d[1]+dist_arm*np.sin(np.deg2rad(deg)), \
                                       0, np.deg2rad(deg), 0]).reshape((6,1))
            # 计算机器人末端执行器提升后的位置和方向角，并重塑为6x1的矩阵
            deg_cloud_lift_pose = np.array([center_2d[0] - dist_arm_lift * np.cos(np.deg2rad(deg)), center[1], center_2d[1] + dist_arm_lift * np.sin(np.deg2rad(deg)), \
                                            0, np.deg2rad(deg), 0]).reshape((6, 1))
            # 将计算出的姿态存储到can_pose_ypr和can_lift_pose_ypr矩阵中
            can_pose_ypr[:, idx:idx + 1] = deg_cloud_pose.copy()
            can_lift_pose_ypr[:, idx:idx + 1] = deg_cloud_lift_pose.copy()

        ## 接下来的代码将只取中间连续的部分
        can_num_half = int((can_num+1)/2) # 计算中间部分的索引

        # 通过迭代找到中间连续部分的最小索引
        min_idx = can_num_half
        for idx in range(can_num_half):
            # 如果当前点的数量大于5，则更新最小索引；否则，退出循环
            idx_in_vec = can_num_half-idx
            if can_pts_num[0, idx_in_vec]>5:
                min_idx = idx_in_vec
            else:
                break
        # 找到中间连续部分的最大索引
        max_idx = can_num_half
        for idx in range(can_num_half):
            # 如果当前点的数量大于5，则更新最大索引；否则，退出循环
            idx_in_vec = can_num_half+idx
            if can_pts_num[0, idx_in_vec]>5:
                max_idx = idx_in_vec
            else:
                break

        # 根据找到的索引，复制有效的机器人姿态
        can_pose_ypr_valid = can_pose_ypr[:, min_idx:max_idx + 1].copy()
        can_lift_pose_ypr_valid = can_lift_pose_ypr[:, min_idx:max_idx + 1].copy()

        # 将提升后的姿态和原始姿态合并，并重塑为一个6行的矩阵，其中每一行代表一个点的位置和方向角
        path_pose = np.block([[can_lift_pose_ypr_valid], [can_pose_ypr_valid]]).reshape((6,-1), order='F')

        return path_pose  # 返回规划的路径姿态

if __name__ == '__main__':
    rospy.init_node('path_plan_server_node')
    server = path_plan_action()
    rospy.spin()