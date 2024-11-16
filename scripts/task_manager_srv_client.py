#!/usr/bin/env python

'''
作为ROS服务的客户端，发送一个任务列表到TaskList服务，并等待响应
'''

#from __future__ import print_function
import sys
import numpy
import rospy
from cashmerebot.srv import *
import numpy as np
from std_msgs.msg import Float32MultiArray
import math

## 定义常量
pi = math.pi

##  初始化任务列表
# 创建一个名为task1的NumPy数组，包含三个任务。
# 每个任务是一个包含10个元素的列表，代表不同的参数。
task1 = numpy.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [2, 0.2, 0, 0, 0, 0, 0, 0, 0, 0],
                     [1, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
# task1 = numpy.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

'''
    定义了一个函数 send_task_list，接受三个参数：
    tasks：一个包含任务的NumPy数组。
    request：一个ROS服务代理对象，用于发送服务请求。
    msg：一个 Float32MultiArray 消息实例，用于封装要发送的数据。
'''
def send_task_list(tasks, request, msg):
    # 使用np.zeros初始化一个20行10列的任务列表数组
    # 假设任务列表的最大长度是20个任务，每个任务有10个参数。
    task_list = np.zeros((20, 10))

    # tasks_num = tasks.shape[0]
    # 将传入的任务列表tasks复制到task_list
    # 因为使用了相同的变量名,覆盖了上一行代码的初始化操作
    task_list = tasks.copy()

    ''' 
     将 task_list 数组展平为一个一维数组，通过 [0] 选择出这个一维数组
     reshape((1, -1)) 重塑为 (1, 200) 的二维数组，其中200是20行10列的总数。
    '''
    task_list_flatten = task_list.reshape((1, -1))[0]
    # 将展平后的数组转换为一个Python列表
    task_list_flatten_list = task_list_flatten.tolist()
    # 将任务数据赋值给消息实例
    msg.data = task_list_flatten_list

    # 记录日志并调用服务请求，打印响应结果
    rospy.loginfo('send TaskList request: ')
    # 通过服务代理 request 发送包含 msg 消息服务的请求，并将响应存储在变量 resp 中
    resp = request(msg)
    # 打印出服务的响应结果
    rospy.loginfo('response is: %s' % (resp))

if __name__ == "__main__":
    # 初始化ROS节点task_manager_srv_client_node
    rospy.init_node('task_manager_srv_client_node', anonymous=True)
    print('starting client')  # 打印启动信息
    rospy.wait_for_service('TaskList') # 等待服务就绪
    print('service connected.')  # 打印服务连接信息

    # 尝试发送任务列表
    try:
        # 创建服务代理
        task_list_request = rospy.ServiceProxy('TaskList', TaskList)
        # 创建消息实例
        msg = Float32MultiArray()
        # 发送任务列表
        send_task_list(task1, task_list_request, msg)
        rospy.sleep(20)
        # 异常处理
    except rospy.ServiceException as e:
        # 打印异常信息
        print("Service call failed: %s" % e)