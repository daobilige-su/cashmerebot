#! /usr/bin/env python

'''
    这个脚本的主要功能：
    1.订阅一个PointCloud2消息
    2.将点云从/front_depth_cam坐标系变换到/ur5_base坐标系
    3.将变换后的点云发布到另一个话题
'''

import numpy as np
import rospy
import sys  # 用于访问与Python解释器相关的变量和函数
import tf   # ROS中用于处理坐标变换的库
from sensor_msgs.msg import PointCloud2  # 用于处理3D点云数据
import ros_numpy
from transform_tools import *

class cloud_transform:
    def __init__(self):
        # 创建一个tf.TransformListener实例，用于监听坐标变换
        self.tf_listener = tf.TransformListener()
        # 创建一个ROS订阅者，订阅/front_depth_cam/points话题，当接收到PointCloud2消息时，调用self.pc2_cb回调函数
        self.pc2_sub = rospy.Subscriber("/front_depth_cam/points", PointCloud2, self.pc2_cb)
        # 创建一个ROS发布者，用于发布话题/front_depth_cam/pc2_ur5_base上的PointCloud2消息，队列大小为5
        self.pc2_pub = rospy.Publisher('/front_depth_cam/pc2_ur5_base', PointCloud2, queue_size=5)

    # 定义了PointCloud2消息的回调函数pc2_cb
    def pc2_cb(self, msg):
        # 将接收到的PointCloud2消息转换为NumPy数组，并转置
        pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg).T

        '''
        解释了如何在ROS中使用tf库来查询坐标帧之间的变换，包括函数的参数、返回值、可能抛出的异常以及如何使用函数
        
        lookupTransform(target_frame, source_frame, time) -> (position, quaternion)
            Parameters:	        
                target_frame – transformation target frame in tf, string
                source_frame – transformation source frame in tf, string
                time – time of the transformation, use rospy.Time() to indicate most recent common time.        
            Returns:	        
                position as a translation (x, y, z) and orientation as a quaternion (x, y, z, w)
            Raises:	        
                tf.ConnectivityException, tf.LookupException, or tf.ExtrapolationException
        
            ## 列出了在某些情况下调用lookupTransform可能触发的异常：
               tf.ConnectivityException：当ROS节点无法连接到tf服务器时抛出。
               tf.LookupException：当变换信息不可用时抛出。
               tf.ExtrapolationException：当请求的变换超出已知的时间范围，需要外推时抛出
        
        Returns the transform from source_frame to target_frame at time. 
        函数的目的是返回从source_frame到target_frame在指定时间的变换
        Raises one of the exceptions if the transformation is not possible. 
        如果变换不可能进行，函数将抛出上述列出的任一异常
        Note that a time of zero means latest common time, so: 
        如果时间参数为零，则表示使用最新的共同时间
        t.lookupTransform("a", "b", rospy.Time())
        展示如何使用lookupTransform函数查询两个坐标帧"a"和"b"之间的变换
        is equivalent to:      #  上下代码等效
        t.lookupTransform("a", "b", t.getLatestCommonTime("a", "b"))
        展示如何使用getLatestCommonTime函数获取两个坐标帧"a"和"b"之间的最新共同时间，并用于lookupTransform
        '''
        try:
            # 查询从'/front_depth_cam'到'/ur5_base'的坐标变换，返回平移向量trans和旋转四元数rot
            (trans, rot) = self.tf_listener.lookupTransform('/ur5_base', '/front_depth_cam', rospy.Time(0))
        # 捕获在查找变换时可能发生的异常
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # 如果发生异常，记录一条警告日志。
            rospy.logwarn('pc2_transform: tf lookup failed')
            return

        # 将平移向量和旋转四元数转换为旋转矩阵M
        M = transform_trans_quat_to_matrix(np.array([[trans[0]], [trans[1]], [trans[2]], [rot[0]], [rot[1]], [rot[2]], [rot[3]]]))
        # 应用旋转矩阵M到点云上，得到变换后的点云
        pc_transformed_np = np.block([[np.eye(3), np.zeros((3, 1))]]) @ M @ np.block([[pc_np], [np.ones((1, pc_np.shape[1]))]])

        # 创建一个新的NumPy数组，用于存储变换后的点云的X、Y、Z坐标
        pc_array = np.zeros(pc_transformed_np.shape[1], dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
        ])

        # 将变换后的点云坐标赋值给新数组
        pc_array['x'] = pc_transformed_np[0, :]
        pc_array['y'] = pc_transformed_np[1, :]
        pc_array['z'] = pc_transformed_np[2, :]

        # 将NumPy数组转换回PointCloud2消息类型，设置时间戳和坐标系
        pc2_transformed_msg = ros_numpy.point_cloud2.array_to_pointcloud2(pc_array, msg.header.stamp, 'ur5_base')
        # 设置变换后的消息序列号与原始消息相同
        pc2_transformed_msg.header.seq = msg.header.seq
        # 发布变换后的PointCloud2消息
        self.pc2_pub.publish(pc2_transformed_msg)

# 定义主函数main，参数args是命令行参数
def main(args):
    rospy.init_node('pc2_transform_node', anonymous=True)
    tfm = cloud_transform()

    rospy.spin()

# 如果直接运行这个脚本，则调用main函数，并传入命令行参数
if __name__ == '__main__':
    main(sys.argv)
