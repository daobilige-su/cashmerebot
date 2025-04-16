#! /usr/bin/env python

'''
用于在ROS（Robot Operating System）中发布坐标变换信息。
坐标变换，通常称为“变换”（TF），用于定义不同坐标系之间的关系
'''

import rospy  # 用于与ROS通信
import sys   # 用于访问与Python解释器相关的变量和函数
import tf    # 用于发布坐标变换

# 用于管理坐标变换的发布
class TFManager:
    def __init__(self):
        # 初始化一个 rospy.Rate 对象以控制循环频率（10Hz）
        self.send_rate = rospy.Rate(10)  # send with 10 hz
        # 初始化一个 tf.TransformBroadcaster 对象用于发布变换
        self.br = tf.TransformBroadcaster()

    # 定义一个方法 tf_broadcast，用于发布坐标变换。
    # 接受四个参数：平移向量 trans（平移），四元数 quat（旋转）,子坐标系 和父坐标系 的名称
    def tf_broadcast(self, trans, quat, child_frame, parent_frame):
        self.br.sendTransform((trans[0], trans[1], trans[2]), (quat[0], quat[1], quat[2], quat[3]), rospy.Time.now(),
                              child_frame, parent_frame)

    # 定义一个方法 send_tf，用于调用 tf_broadcast 方法发送具体的坐标变换
    def send_tf(self):
        # 定义了名为 front_depth_cam 的坐标系相对于其父坐标系 ur5_base 的位置和方向
        self.tf_broadcast([-0.500000, -0.200000, 0.400000], [0.500000, -0.500000, 0.500000, -0.500000],
                          'front_depth_cam', 'ur5_base')
        # 定义了名为 base_link 的坐标系相对于其父坐标系 ur5_base 的位置和方向
        self.tf_broadcast([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0],
                          'base_link', 'ur5_base')
        # 定义了名为 tool_link 的坐标系相对于其父坐标系 tool0 的位置和方向
        self.tf_broadcast([0.0, 0.06, 0.08], [0.0, 0.0, 0.0, 1.0],
                          'tool_link', 'tool0')

def main(args):
    # 初始化一个名为 tf_manager_node 的ROS节点
    rospy.init_node('tf_manager_node', anonymous=True)
    # 创建 TFManager 实例
    tfm = TFManager()

    # 在一个循环中，只要ROS节点没有关闭，就持续调用 send_tf 方法发送坐标变换,根据 send_rate 控制频率
    try:
        while not rospy.is_shutdown():
            tfm.send_tf()
            tfm.send_rate.sleep()
    # 捕获 KeyboardInterrupt 异常（如使用Ctrl+C中断程序），并打印关闭消息
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)   # 调用 main 函数并传入命令行参数
