#! /usr/bin/env python

'''
用于在ROS环境中运行一个名为TaskManager的节点,
这个节点负责管理任务列表,根据任务列表执行路径规划和机械臂操作等动作。
'''

import numpy as np
import cashmerebot.msg
# from __future__ import print_function
# Brings in the SimpleActionClient
import actionlib  # 简化动作客户端的创建
from visualization_msgs.msg import MarkerArray  #  用于可视化

# 导入多种几何消息类型，用于描述机器人的位置和运动
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion, Twist
from visualization_msgs.msg import Marker  # 在ROS中创建可视化标记
import rospy
import sys        # 用于访问与Python解释器相关的变量和函数
from std_msgs.msg import String, Float32MultiArray
from cashmerebot.srv import TaskList, ConvCmd
from tf import transformations   # 用于处理变换

class TaskManager:
    def __init__(self):
        # self.sub = rospy.Subscriber("chatter", String, self.callback)
        # 初始化任务列表为None
        self.task_list = None  # 另一种初始化方式：np.zeros((20, 10))
        # 初始化一个变量，用于跟踪是否已经警告过没有更多任务
        self.no_more_task_warned = 0
        # 创建一个ROS速率对象，用于控制任务执行的频率
        self.task_sleep_rate = rospy.Rate(10)
        # 创建一个名为TaskList的ROS服务，当其他节点请求更新任务列表时，这个服务会调用self.update_task_list方法。
        self.task_list_srv = rospy.Service('TaskList', TaskList, self.update_task_list)

        # 记录一条信息日志，表明TaskList服务已经准备好
        rospy.loginfo('TaskList service ready')
        # 等待名为ConvCmd的服务在ROS系统中注册并可用
        rospy.wait_for_service('ConvCmd')
        # 记录一条信息日志，表明ConvCmd服务已经连接
        rospy.loginfo('ConvCmd service connected.')
        # 创建一个服务代理self.conv_cmd_request，用于调用ConvCmd服务
        self.conv_cmd_request = rospy.ServiceProxy('ConvCmd', ConvCmd)


       #  创建一个动作客户端self.path_plan_client，用于与名为path_plan的动作服务器通信,
       #  该服务器使用cashmerebot.msg.path_planAction定义的动作
        self.path_plan_client = actionlib.SimpleActionClient('path_plan', cashmerebot.msg.path_planAction)
        # 等待path_plan动作服务器启动并准备好接收请求
        self.path_plan_client.wait_for_server()
        # 记录一条信息日志，表明path_plan动作服务器已经连接
        rospy.loginfo('path_plan_server connected.')


        # 创建另一个动作客户端self.manipulation_client，用于与名为manipulation的动作服务器通信,
        # 使用cashmerebot.msg.manipulationAction定义的动作.
        self.manipulation_client = actionlib.SimpleActionClient('manipulation', cashmerebot.msg.manipulationAction)
        # 等待manipulation动作服务器启动并准备好接收请求
        self.manipulation_client.wait_for_server()
        # 记录一条信息日志，表明manipulation动作服务器已经连接
        rospy.loginfo('manipulation_server connected.')

        # 记录一条警告日志，提示系统将在3秒后开始
        rospy.logwarn('Going to Start after 3s ...')
        rospy.sleep(3)
        rospy.logwarn('Started!')  # 记录一条警告日志，表明系统已经启动

    # update_task_list方法用于接收新的任务列表，更新内部任务列表状态，
    # 取消所有正在进行的路径规划任务，并准备执行新的任务。

    # 定义一个服务回调函数，用于处理来自TaskList服务的请求。
    def update_task_list(self, req):   # req是服务请求对象，包含客户端发送的数据。
        # 将请求中的数据转换为NumPy数组，并将其重塑为一个10列的二维数组。req.list.data是一个包含任务列表数据的列表。
        task_list = np.array(req.list.data).reshape((-1, 10))
        # 记录一条信息日志，说明收到了新的任务列表
        rospy.loginfo('received new task_list: ')
        # 打印出接收到的任务列表内容
        rospy.loginfo(task_list)
        # 将新的任务列表复制给实例变量self.task_list。
        # 使用.copy()确保复制的是数组数据而不是引用，防止外部对任务列表的修改影响到类的内部状态。
        self.task_list = task_list.copy()
        # 取消path_plan_client动作客户端的所有目标。
        # 通常在接收到新任务列表时执行，确保之前的路径规划任务不会与新任务冲突。
        self.path_plan_client.cancel_all_goals()
        # rospy.sleep(0.5)
        self.stop()  # 发送停止命令
        return True

    '''
    定义了TaskManager类中的execute_task方法，负责从任务列表中取出任务并执行相应的操作
    目的是按顺序执行任务列表中的任务，并在任务完成后更新任务列表的状态。 
    '''
    def execute_task(self):
        # 检查self.task_list（任务列表）是否已经被初始化，即它不是None
        if self.task_list is not None:
            # 重置no_more_task_warned标志为0，记录是否已经警告过没有更多任务
            self.no_more_task_warned = 0
            # 获取任务列表中的行数（任务数量）
            task_num = self.task_list.shape[0]
            # 从任务列表中取出第一个任务，并复制它，以便在处理时不会影响原始列表。
            task_list_cur = self.task_list[0, :].copy()
            # 记录一条警告日志，显示当前正在执行的任务
            rospy.logwarn('Executing task: ')
            rospy.logwarn(task_list_cur)


            # 如果任务列表中只剩下一个任务，执行完后将其设置为None；
            # 否则，删除已执行的任务，只保留剩余的任务。
            if task_num == 1:
                self.task_list = None
            else:
                self.task_list = self.task_list[1:task_num, :].copy()

            if task_list_cur[0] == 0:   # 如果当前任务的第一个元素是0，表示这是一个停止模式的任务。
                self.path_plan_client.cancel_all_goals()   # 取消所有路径规划客户端的目标
                self.stop() # publish all zero velocity cmd    # 调用stop方法，停止机器人的运动

            elif task_list_cur[0] == 1:            # 如果当前任务的第一个元素是1，表示这是一个路径规划模式的任务
                params = task_list_cur[1:3].copy()       # 提取任务参数，这里只复制了x和y坐标
                path_np = self.path_plan_action(params)  # 调用path_plan_action方法进行路径规划，传入参数
                self.manipulation_action(path_np)        # 调用manipulation_action方法执行操作，传入规划好的路径

            elif task_list_cur[0] == 2:          # 如果当前任务的第一个元素是2，表示这是一个传送带运动模式的任务
                self.conveyor_motion_request(task_list_cur[1]) # 请求传送带运动到指定位置
            else:
                rospy.logerr('unknown task code.')

        else:
            if not self.no_more_task_warned:            # 如果任务列表是None，并且之前没有警告过，
                rospy.logwarn('Task list empty now.')   # 记录一条警告日志表示任务列表现在是空的
                self.no_more_task_warned = 1            # 将no_more_task_warned标志设置为1，以避免重复警告

    # move_dir:
    # 1. move_base task: 0, stay still; 1, move forward; 2, move backward; 3, move left; 4, move right
    # 5, move forward no jump
    # 9, turn 180

    '''发送路径规划请求并处理结果'''
    # 定义了一个名为path_plan_action的方法，接受两个参数:self和params(包含路径规划参数的列表或数组)。
    def path_plan_action(self, params):
        # 创建了一个path_planGoal消息实例，用于与路径规划动作服务器通信的目标消息类型
        goal = cashmerebot.msg.path_planGoal()
        # 将传入的参数params转换为列表，并赋值给目标消息的params属性
        goal.params = params.tolist()
        # 通过path_plan_client动作客户端发送路径规划目标
        self.path_plan_client.send_goal(goal)
        # 记录一条警告日志，显示发送了新的目标，其中包含目标的前两个参数，x和y坐标
        rospy.logwarn('path_plan_client: sent new goal (%f, %f)' % (goal.params[0], goal.params[1]))
        # 等待路径规划动作客户端的结果
        self.path_plan_client.wait_for_result()
        # 当收到结果后，记录一条警告日志，表示目标已完成
        rospy.logwarn("path_plan_client: goal completed")
        # 获取路径规划动作客户端的结果
        path_list = self.path_plan_client.get_result()

        # 将结果中的路径转换为NumPy数组，并使用reshape将其重塑为一个6行数组，列数根据路径点的数量动态确定。
        # order='F'表示按照Fortran顺序（列优先）来填充数组
        path_np = np.array(path_list.path).reshape((6,-1), order='F')
        # 返回处理后的路径数组path_np
        return path_np

    '''用于执行与机械臂操作相关的动作'''
    #定义了一个名为manipulation_action的方法，接受两个参数：self和path_np(一个包含机械臂路径点的NumPy数组）。
    def manipulation_action(self, path_np):
        # 用于与机械臂操作动作服务器通信的目标消息类型
        goal = cashmerebot.msg.manipulationGoal()

        # 将传入的路径数组path_np进行转置，然后使用reshape将其变为一维数组，并转换为列表，赋值给目标消息的path属性。
        # 转置是为了将路径点的顺序从行优先（Python默认）转换为列优先
        goal.path = path_np.T.reshape((-1,)).tolist()

        # 通过manipulation_client动作客户端发送机械臂操作目标
        self.manipulation_client.send_goal(goal)
        # 记录一条警告日志，表示已发送新的路径给机械臂操作客户端
        rospy.logwarn('manipulation_client: sent new path')
        # 等待机械臂操作动作客户端的结果
        self.manipulation_client.wait_for_result()
        # 当收到结果后，记录一条警告日志，表示目标已完成
        rospy.logwarn("manipulation_client: goal completed")

    # 发送停止命令到机器人或系统的某个部分
    def stop(self):
        pass

    '''
    创建了一个消息实例来封装传送带的目标位置，然后通过服务代理发送这个请求，并记录服务的响应。
    目的:控制传送带移动到指定的位置
    '''
    # 定义了一个名为conveyor_motion_request的方法，接受两个参数：self和pos（一个数值，表示传送带的目标位置）。
    def conveyor_motion_request(self, pos):
        # 创建了一个新的Float32MultiArray消息实例，是ROS中用于传递浮点数数组的标准消息类型
        msg = Float32MultiArray()
        msg.data = [pos]   # 将目标位置pos添加到消息的数据数组中
        # 记录一条信息日志，表示即将发送传送带命令请求
        rospy.loginfo('send ConvCmd request: ')
        # 使用之前创建的conv_cmd_request服务代理，发送包含位置信息的请求消息，并存储响应结果到变量resp中。
        resp = self.conv_cmd_request(msg)
        # 记录一条信息日志，打印出服务的响应结果
        rospy.loginfo('response is: %s' % (resp))

# Python脚本的主体部分，定义了一个ROS节点，该节点会持续运行并执行任务，直到被外部中断
# 定义了一个名为main的函数，接受参数args(代表命令行参数)
def main(args):
    # 初始化一个名为task_manager_node的ROS节点
    # anonymous=True参数使得每次启动节点时都有一个随机生成的名称后缀，以避免名称冲突。
    rospy.init_node('task_manager_node', anonymous=True)
    # 创建了TaskManager类的一个实例，命名为tm
    tm = TaskManager()
    try:
        while not rospy.is_shutdown():  # 用于检查ROS系统是否已经关闭
            tm.execute_task()   # 调用tm实例的execute_task方法来执行任务
            rospy.sleep(0.2) # 每次循环迭代后，使程序暂停0.2秒
    except KeyboardInterrupt:    # 定义了一个except块，用于捕获KeyboardInterrupt异常
        print("Shutting down")

# 用于判断如果这个脚本是作为主程序运行，调用main函数，并传入命令行参数sys.argv
if __name__ == '__main__':
    main(sys.argv)