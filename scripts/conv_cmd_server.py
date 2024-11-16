#! /usr/bin/env python
'''
代码功能：
1.创建一个ROS服务节点，该节点提供ConvCmd服务，用于接收传送带位置的命令,并将这些命令通过ROS消息发布出去
2.将其转换为Float32MultiArray消息并发布，以便其他节点订阅这些消息后执行相应动作
'''

import numpy as np
import rospy
import sys
from std_msgs.msg import String, Float32MultiArray
from cashmerebot.srv import ConvCmd

class ConvCmdServer:
    def __init__(self):

        # self.task_sleep_rate = rospy.Rate(10)

        #self.task_list_srv = rospy.Service(服务名称，服务类型，服务回调函数）允许节点之间进行一次性的请求-响应通信
        self.task_list_srv = rospy.Service('ConvCmd', ConvCmd, self.conv_cmd_callback)

        rospy.loginfo('ConvCmd service ready') #打印日志信息

        # 创建一个名为'ConvCmd_msg'的发布者，用于发布Float32MultiArray类型的消息，队列大小为2。
        self.conv_cmd_pub = rospy.Publisher('ConvCmd_msg', Float32MultiArray, queue_size=2)

    # send conveyor position cmd via ros msg
    # 服务回调函数，当服务请求被调用时执行
    def conv_cmd_callback(self, req):
        # 将请求中的列表数据转换为numpy数组，并重塑为1维数组
        cmd_list = np.array(req.list.data).reshape((1,))
        # 记录日志信息，显示接收到的新命令
        rospy.loginfo('received new cmd: ')
        rospy.loginfo(cmd_list)

        msg = Float32MultiArray()  # 创建一个空的Float32MultiArray消息
        # 将numpy数组转换为列表，并赋值给消息的数据字段。
        msg.data = cmd_list.tolist()
        self.conv_cmd_pub.publish(msg)

        rospy.sleep(1.0)
        return True

def main(args):
    # 初始化名为'conv_cmd_server_node'的ROS节点
    rospy.init_node('conv_cmd_server_node', anonymous=True)

    # 创建ConvCmdServer类的实例
    srv = ConvCmdServer()
    try:
        rospy.spin()  # 保持节点运行，等待回调函数被调用
    except KeyboardInterrupt:
        print("Shutting down")

# 程序入口，调用main函数并传入命令行参数
if __name__ == '__main__':
    main(sys.argv)
