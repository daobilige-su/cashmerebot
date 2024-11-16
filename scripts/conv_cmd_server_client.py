#!/usr/bin/env python

'''该脚本是一个ROS客户端，用于发送任务列表到一个名为ConvCmd的服务，并等待响应'''

from __future__ import print_function #从Python 2.x版本中导入print函数，使得在Python 3.x中也可以使用
import sys
import numpy
import rospy
from cashmerebot.srv import *  #从cashmerebot包的srv模块中导入了所有内容
import numpy as np
from std_msgs.msg import Float32MultiArray
import math

pi = math.pi  #定义变量pi，并赋值为数学常数π

# task1 = numpy.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                      [1, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

#定义一个numpy数组task1，初始值为一个包含单个元素1的数组
task1 = numpy.array([[1]])

# 定义一个函数send_task_list，接收三个参数：tasks是任务列表，request是服务代理对象，msg是消息对象
def send_task_list(tasks, request, msg):
    cmd_list = tasks.copy()   # 将tasks数组复制并赋值给cmd_list变量
    cmd_list_flatten = cmd_list.reshape(1,) # .reshape改变数组的形状而不改变其数据，将cmd_list数组变为一维数组
    cmd_list_flatten_list = cmd_list_flatten.tolist()  # .tolist()将cmd_list_flatten数组转换为一个Python列表
    msg.data = cmd_list_flatten_list  # 将这个列表赋值给msg.data

    rospy.loginfo('send ConvCmd request: ')  # 打印日志信息
    resp = request(msg)  # 调用服务代理对象的请求方法
    rospy.loginfo('response is: %s' % (resp))  # 将消息对象作为参数传递

if __name__ == "__main__":
    rospy.init_node('conv_cmd_server_client_node', anonymous=True)
    print('starting client') # 打印信息表明客户端正在启动
    rospy.wait_for_service('ConvCmd') # 等待名为ConvCmd的服务可用
    print('service connected.') # 打印信息表明服务已连接

    try:
        task_list_request = rospy.ServiceProxy('ConvCmd', ConvCmd) # 使用ConvCmd服务类型，尝试创建一个服务代理对象task_list_request
        msg = Float32MultiArray()  # 创建一个Float32MultiArray类型的消息对象
        send_task_list(task1, task_list_request, msg)  # 调用send_task_list函数发送任务列表
        rospy.sleep(20)

    # 捕获并处理rospy.ServiceException异常，打印错误信息
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)