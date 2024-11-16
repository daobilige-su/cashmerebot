#! /usr/bin/env python

'''
这个脚本的功能是在ROS环境中管理一系列任务，包括：停止机器人、请求路径规划等。
通过服务接收任务列表更新，通过动作客户端与路径规划服务器交互，并在一个持续的循环中执行这些任务。
'''

import numpy as np
import cashmerebot.msg
import rospy
# from __future__ import print_function
# Brings in the SimpleActionClient
import actionlib
from visualization_msgs.msg import MarkerArray

# 从geometry_msgs.msg模块导入多个类，用于定义几何消息，如姿态、姿态数组、点、四元数和线速度。
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion, Twist
# 用于创建和修改可视化标记
from visualization_msgs.msg import Marker
import rospy
import sys
# 用于字符串和32位浮点数数组
from std_msgs.msg import String, Float32MultiArray
# 用于处理任务列表
from cashmerebot.srv import TaskList
# 用于处理坐标变换
from tf import transformations

class TaskManager:
    def __init__(self):
        # self.sub = rospy.Subscriber("chatter", String, self.callback)
        # 初始化一个名为task_list的实例变量，初始值为None
        self.task_list = None  # np.zeros((20, 10))
        # 用于记录是否已经警告过没有更多任务
        self.no_more_task_warned = 0
        # 创建一个ROS速率对象，设置为10Hz，用于控制循环的执行频率
        self.task_sleep_rate = rospy.Rate(10)

        # TaskList service to update self.task_list
        # 创建一个名为task_list_srv的ROS服务
        # 服务名称为TaskList，服务类型为TaskList，服务处理函数为self.update_task_list
        self.task_list_srv = rospy.Service('TaskList', TaskList, self.update_task_list)
        # 表示TaskList服务已准备好
        rospy.loginfo('TaskList service ready')

        # path_plan client
        # 创建一个动作客户端path_plan_client，与path_plan动作服务器交互，动作类型为cashmerebot.msg.path_planAction
        self.path_plan_client = actionlib.SimpleActionClient('path_plan', cashmerebot.msg.path_planAction)
        # 等待path_plan动作服务器启动
        self.path_plan_client.wait_for_server()
        # 表示已连接到path_plan_server
        rospy.loginfo('path_plan_server connected.')

        # ready to start
        # 记录一条警告级别的日志，表示将在3秒后开始
        rospy.logwarn('Going to Start after 3s ...')
        rospy.sleep(3)
        # 记录一条警告级别的日志，表示已经开始执行
        rospy.logwarn('Started!')

    '''接收一个任务列表，更新内部的任务列表状态，取消所有已发送的路径规划目标，并可能停止当前的任务处理'''
    # 定义了TaskManager类的一个方法，接受两个参数：self（类的实例）和req（服务请求对象）
    def update_task_list(self, req):
        # 将服务请求中的list.data字段转换成NumPy数组，并重新排列为一个10列的二维数组。
        # -1表示自动计算行数以匹配数组中的元素总数
        task_list = np.array(req.list.data).reshape((-1, 10))
        # 记录一条信息级别的日志，表示接收到了新的任务列表
        rospy.loginfo('received new task_list: ')
        # 记录接收到的任务列表内容
        rospy.loginfo(task_list)

        # 将task_list数组的副本赋值给实例变量self.task_list。
        # 使用copy()是为了避免直接引用，确保self.task_list是数据的一个独立副本。
        self.task_list = task_list.copy()
        # 调用动作客户端，取消所有已发送给path_plan动作服务器的目标
        self.path_plan_client.cancel_all_goals()

        # rospy.sleep(0.5)
        self.stop()  # 停止当前的任务
        return True  # 表示服务请求已被成功处理

    def execute_task(self):
        # 检查是否有任务需要执行
        if self.task_list is not None:
            # 如果任务列表不为空，则重置no_more_task_warned变量为0，这可能用于控制警告消息的显示次数。
            self.no_more_task_warned = 0

            # 获取任务列表的行数，即任务的数量
            task_num = self.task_list.shape[0]
            # 获取当前要执行的任务（任务列表的第一行），并复制到task_list_cur变量中
            task_list_cur = self.task_list[0, :].copy()
            # 记录一条警告级别的日志，表示开始执行任务
            rospy.logwarn('Executing task: ')
            # 记录当前要执行的任务的详细信息
            rospy.logwarn(task_list_cur)


            if task_num == 1:          # 如果任务列表中只有一个任务，
                self.task_list = None  # 将任务列表设置为None，表示所有任务已被执行

            # 如果任务列表中有多个任务，移除任务列表中的第一个任务，并复制剩余任务到self.task_list
            else:
                self.task_list = self.task_list[1:task_num, :].copy()

            if task_list_cur[0] == 0:  # stop mode, [0, ...]   # 如果当前任务的第一个元素是0，
                self.path_plan_client.cancel_all_goals()       # 取消路径规划客户端的所有目标
                # publish all zero velocity cmd                # 停止机器人的运动
                self.stop()

            elif task_list_cur[0] == 1:  # path plan mode, [1, x, y, theta, ...] #  如果当前任务的第一个元素是1
                params = task_list_cur[1:3].copy()  # 提取任务参数（从第二个元素到第三个元素），并复制到params变量中
                path_np = self.path_plan_action(params)  #执行路径规划

            # 如果当前任务的第一个元素既不是0也不是1，录一条错误级别的日志，表示遇到了未知的任务代码
            else:
                rospy.logerr('unknown task code.')

        else:
            if not self.no_more_task_warned:              # 没有任务要执行，如果之前没有警告过任务列表为空，
                rospy.logwarn('Task list empty now.')     # 记录一条警告级别的日志
                self.no_more_task_warned = 1          # 将no_more_task_warned设置为1，表示已经警告过任务列表为空

    # move_dir:
    # 1. move_base task: 0, stay still; 1, move forward; 2, move backward; 3, move left; 4, move right
    # 5, move forward no jump
    # 9, turn 180

    '''
    创建一个路径规划的目标，发送给动作服务器，并等待结果。
    一旦结果返回，它将结果转换为NumPy数组，并按照特定的形状重新排列，最后返回这个数组。
    '''
    def path_plan_action(self, params):
        goal = cashmerebot.msg.path_planGoal()
        # 将传入的params参数（可能是一个NumPy数组）转换为列表，并赋值给目标消息的params属性
        goal.params = params.tolist()

        # 使用path_plan_client动作客户端发送目标消息给路径规划动作服务器
        self.path_plan_client.send_goal(goal)
        # 记录一条警告级别的日志，显示发送给路径规划服务器的新目标的参数
        rospy.logwarn('path_plan_client: sent new goal (%f, %f)' % (goal.params[0], goal.params[1]))
        # 调用动作客户端的wait_for_result方法，等待动作服务器返回结果
        self.path_plan_client.wait_for_result()
        # 一旦动作服务器返回结果，记录一条警告级别的日志，表示目标已完成
        rospy.logwarn("path_plan_client: goal completed")

        # 从动作客户端获取结果，存储在path_list变量中
        path_list = self.path_plan_client.get_result()

        # 将获取的结果中的路径部分转换为NumPy数组，并使用reshape方法将其重新排列为一个6行的数组，列数由路径长度决定
        # order='F'表示按照Fortran顺序（列优先）来重新排列数组
        path_np = np.array(path_list.path).reshape((6,-1), order='F')

        # 返回重新排列后的路径数组path_np
        return path_np

    def stop(self):
        pass

'''
初始化一个ROS节点，创建任务管理器实例，并在一个循环中不断执行任务，直到系统关闭或用户中断。
'''
def main(args):
    rospy.init_node('task_manager_perception_only_node', anonymous=True)
    tm = TaskManager()
    try:
        while not rospy.is_shutdown():  # 使用一个循环，只要ROS系统没有关闭，就不断执行
            tm.execute_task()       # 在循环内部，调用tm实例的execute_task方法，执行任务管理
            rospy.sleep(0.2)
    except KeyboardInterrupt:   # 捕获KeyboardInterrupt异常，通常发生在用户按下Ctrl+C时
        print("Shutting down")  # 如果捕获到KeyboardInterrupt异常，打印出"Shutting down"消息，表示程序正在关闭

if __name__ == '__main__':
    main(sys.argv)