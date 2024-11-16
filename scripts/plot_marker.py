#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
这段代码定义了三个函数，用于在ROS的RViz工具中绘制不同类型的标记
这些标记可以用来可视化点（pts）、轨迹（traj）、和箭头（arrows）
通常用于展示机器人路径规划的结果或移动的参考坐标
'''

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from visualization_msgs.msg import Marker
from transform_tools import *

# 定义函数plot_pts，参数包括点的Nx3数组pts，发布者marker_pub，点的ID，坐标帧frame_id和点的尺寸scale_size
def plot_pts(pts, marker_pub, id, frame_id='map', scale_size=0.01): # pts: Nx3
    # 创建一个MarkerArray对象，用于存储一系列的标记
    marker_array = MarkerArray()
    marker_array.markers = []

    ## pts
    pt_marker = Marker()   # 创建一个Marker对象，用于定义一个点的标记。
    pt_marker.header.frame_id = frame_id   # 指定了坐标帧的ID，在这个坐标帧中绘制点
    pt_marker.ns = "path_plan_" + "pt"  # 设置Marker对象的命名空间（ns）
    pt_marker.id = id   # 为Marker对象设置一个唯一的ID，作为识别不同标记的依据
    pt_marker.type = Marker.CUBE_LIST    # 设置标记类型为CUBE_LIST，表示一系列立方体（点）
    pt_marker.action = Marker.ADD # 设置Marker对象的动作为ADD，当这个对象被发布时，被添加到现有的标记集合中

    # 创建一个Pose对象，用来设置点的初始位置和方向
    pose = Pose()
    # 设置Pose对象的四元数方向，w=1.0表示没有旋转，点的方向与坐标系的正方向一致
    pose.orientation.w = 1.0
    # 将Pose对象赋值给Marker对象的pose属性，设置点的位置和方向
    pt_marker.pose = pose
    # when list is used, color needs to be 1.0 not 255, such a bug!
    # 设置Marker对象的颜色为黄色，使用四元组颜色值，范围从0到1，而不是255
    pt_marker.color.r, pt_marker.color.g, pt_marker.color.b = (1.0, 1.0, 0)
    # 设置Marker对象颜色的透明度，1.0表示完全不透明
    pt_marker.color.a = 1.0
    # 设置Marker对象的尺寸，scale_size定义了点的大小。由于是立方体，X、Y、Z三个方向上的尺寸都是相同的
    pt_marker.scale.x, pt_marker.scale.y, pt_marker.scale.z = (scale_size, scale_size, scale_size)
    # 初始化一个空列表，用于存储Point对象，这些对象定义了点的位置
    pt_marker.points = []

    # pt_marker.colors = []
    # 获取传入的点集合pts的行数（点的数量），用于后续循环中创建点
    pts_num = pts.shape[0]

    # 循环遍历点的数组，为每个点创建一个Point对象，并添加到标记的points列表中
    # 开始一个循环，使用i作为索引，遍历pts_num次，pts_num是点集合pts的点数
    for i in range(pts_num):
        # 在循环内部，创建一个新的Point对象，Point是ROS中用于表示三维空间中点的一个消息类型
        pt = Point()
        # 为新创建的Point对象设置X、Y、Z坐标值，这些值从点集合pts的二维NumPy数组中获取
        # 其中i是点的索引，0、1、2分别代表点的X、Y、Z坐标
        pt.x = pts[i, 0]
        pt.y = pts[i, 1]
        pt.z = pts[i, 2]
        # 将设置了坐标的Point对象添加到pt_marker的points列表中，这个列表存储所有点的坐标，在RViz中显示
        pt_marker.points.append(pt)

        # color = ColorRGBA()
        # color.r, color.g, color.b = (255, 255, 255)
        # color.a = 1.0
        # pt_marker.colors.append(color)
    # 将填充了点坐标的pt_marker对象添加到marker_array的markers列表中。
    # marker_array是一个MarkerArray对象，用于存储多个标记
    marker_array.markers.append(pt_marker)
    # 使用ROS发布者marker_pub发布marker_array对象
    marker_pub.publish(marker_array)

# 定义了一个名为 plot_traj 的函数，其功能是在ROS的RViz工具中绘制轨迹点和轨迹线
def plot_traj(traj_pts, marker_pub, start_id, frame_id='map', scale_size=0.01):
    '''
    traj_pts: 一个Nx3的数组，包含了轨迹点的坐标。
    marker_pub: 一个ROS发布者对象，用于发布标记。
    start_id: 起始ID，用于标记的识别。
    frame_id: 坐标帧ID，默认为 'map'。
    scale_size: 标记的尺寸，默认为 0.01
    '''
    # 创建一个 MarkerArray 对象，用于存储一系列的标记对象
    marker_array = MarkerArray()
    marker_array.markers = []

    ## traj_pts（点）
    # 创建了一个新的 Marker 对象，这个对象用于定义要在RViz中绘制的标记
    pt_marker = Marker()
    # 指定了该标记所处坐标帧的ID
    pt_marker.header.frame_id = frame_id
    # 设置了 Marker 对象的命名空间（ns）
    pt_marker.ns = "path_plan_" + "traj_pts"
    # 为 Marker 对象设置了一个唯一的ID
    pt_marker.id = start_id
    # 设置了 Marker 对象的类型为 CUBE_LIST，表示一系列立方体点
    pt_marker.type = Marker.CUBE_LIST
    # 设置了 Marker 对象的动作为 ADD，当这个标记被发布时，被添加到现有的标记集合中
    pt_marker.action = Marker.ADD
    # 创建了一个新的 Pose 对象用于设置标记的位置和方向
    pose = Pose()
    # 设置了 Pose 对象的四元数方向，w =1.0 表示没有旋转，标记的方向与坐标系的正方向一致
    pose.orientation.w = 1.0
    # 将 Pose 对象赋值给 Marker 对象的 pose 属性，设置标记的位置和方向
    pt_marker.pose = pose
    # when list is used, color needs to be 1.0 not 255, such a bug!
    # 设置了 Marker 对象的颜色为红色，颜色分量的值是从0到1的浮点数
    pt_marker.color.r, pt_marker.color.g, pt_marker.color.b = (1.0, 0.0, 0)
    # 设置了 Marker 对象颜色的透明度，1.0 表示完全不透明
    pt_marker.color.a = 1.0
    # 设置了 Marker 对象的尺寸，将每个维度的尺寸设置为 scale_size函数参数 的2倍
    pt_marker.scale.x, pt_marker.scale.y, pt_marker.scale.z = (scale_size*2, scale_size*2, scale_size*2)
    # 初始化了一个空列表 points，用于存储 Point 对象，这些对象定义了每个点在空间中的位置
    pt_marker.points = []

    # pt_marker.colors = []
    # 获取了传入的轨迹点数组 traj_pts 的长度，即点的数量
    traj_pts_num = traj_pts.shape[0]
    # 开始一个循环，使用变量 i 作为索引，从0到 traj_pts_num轨迹点数组中的点的数量 - 1
    for i in range(traj_pts_num):
        # 在循环内部，为当前轨迹点创建一个新的 Point 对象。
        # Point 是ROS中用于表示三维空间中的点的消息类型
        pt = Point()
        # 从 traj_pts 数组中获取第 i 个点的X、Y、Z坐标，并分别赋值给 Point 对象的 x、y、z 属性
        pt.x = traj_pts[i, 0]
        pt.y = traj_pts[i, 1]
        pt.z = traj_pts[i, 2]
        # 将包含当前点坐标的 Point 对象添加到 pt_marker 的 points 列表中。
        # 这个列表最终将包含所有的轨迹点，用于在RViz中显示
        pt_marker.points.append(pt)

        # color = ColorRGBA()
        # color.r, color.g, color.b = (255, 255, 255)
        # color.a = 1.0
        # pt_marker.colors.append(color)

    # 在循环结束后，将包含所有轨迹点的 Marker 对象 pt_marker 添加到 MarkerArray 对象 marker_array 的 markers 列表中。
    # MarkerArray 是一个包含了多个 Marker 对象的数组，它将被用来在RViz中显示所有的标记
    marker_array.markers.append(pt_marker)

    ## traj line（线）
    # 创建一个新的 Marker 对象，用于定义在RViz中绘制的线
    line_marker = Marker()
    # 指定该标记所处坐标帧的ID
    line_marker.header.frame_id = frame_id
    # 设置 Marker 对象的命名空间（ns）
    line_marker.ns = "path_plan_" + "traj_line"
    # 为 Marker 对象设置了一个唯一的ID
    line_marker.id = start_id+1
    # 设置 Marker 对象的类型为 LINE_STRIP，表示一系列连续的线段
    line_marker.type = Marker.LINE_STRIP
    #设置 Marker 对象的动作为 ADD，当这个标记被发布时，被添加到现有的标记集合中。
    line_marker.action = Marker.ADD
    # 创建一个新的 Pose 对象，用于设置标记的位置和方向
    pose = Pose()
    # 设置 Pose 对象的四元数方向，w=1 表示没有旋转，标记的方向与坐标系的正方向一致。
    pose.orientation.w = 1
    # 将 Pose 对象赋值给 Marker 对象的 pose 属性，设置线的位置和方向
    line_marker.pose = pose
    # 设置 Marker 对象的颜色为红色
    line_marker.color.r, line_marker.color.g, line_marker.color.b = (1.0, 0.0, 0.0)
    # 设置 Marker 对象颜色的透明度，1.0 表示完全不透明
    line_marker.color.a = 1.0
    # 设置 Marker 对象的尺寸，将每个维度的尺寸设置为 scale_size，定义了线的粗细
    line_marker.scale.x, line_marker.scale.y, line_marker.scale.z = (scale_size, scale_size, scale_size)

    # 初始化一个空列表 points，用于存储 Point 对象，这些对象定义了线段上的点的位置
    line_marker.points = []

    # 开始一个循环，使用变量 i 作为索引，从0到 traj_pts_num - 1
    # traj_pts_num 是轨迹点数组中的点的数量
    for i in range(traj_pts_num):
        # 在循环内部，为当前轨迹点创建一个新的 Point 对象
        pt = Point()
        # 从 traj_pts 数组中获取第 i 个点的X、Y、Z坐标，并分别赋值给新创建的 Point 对象的 x、y、z 属性。
        pt.x = traj_pts[i, 0]
        pt.y = traj_pts[i, 1]
        pt.z = traj_pts[i, 2]

        # 将包含当前点坐标的 Point 对象添加到 line_marker 的 points 列表中。
        # 这个列表将包含所有的轨迹点，用于在RViz中形成一条连续的线。
        line_marker.points.append(pt)

    # 在循环结束后，将包含轨迹线的 Marker 对象 line_marker 添加到 MarkerArray 对象 marker_array 的 markers 列表中。
    marker_array.markers.append(line_marker)

    # 使用提供的 marker_pub 发布者发布 MarkerArray 对象 marker_array。
    # 在ROS中发布所有的标记，可以在RViz中被可视化
    marker_pub.publish(marker_array)


'''
定义了函数 plot_arrows，它接受以下参数：
    poses: 一个Nx6的数组，其中N是姿态的数量，6代表每个姿态有6个数值（通常是x, y, z坐标和三个欧拉角或四元数）。
    marker_pub: 一个ROS发布者对象，用于发布标记。
    start_id: 起始ID，用于标记的识别。
    frame_id: 坐标帧ID，默认为 'map'。
    scale_size: 标记的尺寸，默认为 0.01。
'''
def plot_arrows(poses, marker_pub, start_id, frame_id='map', scale_size=0.01):
    # 创建一个 MarkerArray 对象，用于存储一系列的标记对象，并初始化其 markers 列表。
    marker_array = MarkerArray()
    marker_array.markers = []

    # 获取 poses 数组的长度，即姿态的数量
    poses_num = poses.shape[0]

    # 开始一个循环，使用变量 n 作为索引，从0到 poses_num - 1
    # poses_num 是姿态数组中的元素数量。
    for n in range(poses_num):
        # arrow
        # 在循环内部，为当前姿态创建一个新的 Marker 对象，用于表示一个箭头
        arrow_marker = Marker()
        # 指定该标记所处坐标帧的ID
        arrow_marker.header.frame_id = frame_id
        # 设置箭头标记的命名空间（ns），用于区分不同的标记集合。
        arrow_marker.ns = "path_plan_" + "arrow"
        # 为箭头标记设置一个唯一的ID，这个ID是 start_id 加上当前索引 n。
        arrow_marker.id = start_id+n
        # 设置箭头标记的类型为 ARROW，表示它将在RViz中显示为一个箭头
        arrow_marker.type = Marker.ARROW
        # 设置箭头标记的动作为 ADD，当这个标记被发布时，被添加到现有的标记集合中。
        arrow_marker.action = Marker.ADD

        # pose
        # 从 poses 数组中提取第 n 个姿态的位置信息（x, y, z），并重塑为一维数组。
        pose_trans = poses[n, 0:3].reshape((-1,))
        # 从 poses 数组中提取第 n 个姿态的欧拉角（偏航-俯仰-翻滚，YPR），并重塑为一维数组。
        pose_ypr = poses[n, 3:6].reshape((-1,))
        # 将提取的欧拉角转换为四元数,并重塑为一维数组。
        pose_quat = ypr2quat(pose_ypr).reshape((-1,))

        # 创建一个新的 Pose 对象，用于设置箭头标记的位置和方向。
        pose = Pose()
        # 设置 Pose 对象的位置信息。
        pose.position.x = pose_trans[0]
        pose.position.y = pose_trans[1]
        pose.position.z = pose_trans[2]
        # 设置 Pose 对象的方向信息，使用之前转换得到的四元数。
        pose.orientation.x = pose_quat[0]
        pose.orientation.y = pose_quat[1]
        pose.orientation.z = pose_quat[2]
        pose.orientation.w = pose_quat[3]
        # 将 Pose 对象赋值给 Marker 对象的 pose 属性。
        arrow_marker.pose = pose

        # when list is used, color needs to be 1.0 not 255, such a bug!
        # 设置箭头标记的颜色为红色。
        arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b = (1.0, 0.0, 0)
        # 设置箭头标记颜色的透明度为完全不透明。
        arrow_marker.color.a = 1.0
        # 设置箭头标记的尺寸，其中 scale.x 被设置为其他两个尺寸的5倍，以确保箭头的杆比头更长。
        arrow_marker.scale.x, arrow_marker.scale.y, arrow_marker.scale.z = (scale_size * 5, scale_size, scale_size)
        # 将创建的箭头标记添加到 MarkerArray 对象的 markers 列表中。
        marker_array.markers.append(arrow_marker)

    # 使用 marker_pub 发布者发布 MarkerArray 对象，使得所有的箭头标记在ROS中的主题上发布，在RViz中显示。
    marker_pub.publish(marker_array)