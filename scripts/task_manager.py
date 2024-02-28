#! /usr/bin/env python

import numpy as np
import cashmerebot.msg
import rospy
# from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion, Twist
from visualization_msgs.msg import Marker
import rospy
import sys
from std_msgs.msg import String, Float32MultiArray
from cashmerebot.srv import TaskList
from tf import transformations


class TaskManager:
    def __init__(self):
        # self.sub = rospy.Subscriber("chatter", String, self.callback)
        self.task_list = None  # np.zeros((20, 10))
        self.no_more_task_warned = 0
        self.task_sleep_rate = rospy.Rate(10)

        # TaskList service to update self.task_list
        self.task_list_srv = rospy.Service('TaskList', TaskList, self.update_task_list)
        rospy.loginfo('TaskList service ready')

        # path_plan client
        self.path_plan_client = actionlib.SimpleActionClient('path_plan', cashmerebot.msg.path_planAction)
        self.path_plan_client.wait_for_server()
        rospy.loginfo('path_plan_server connected.')

        # ready to start
        rospy.logwarn('Going to Start after 3s ...')
        rospy.sleep(3)
        rospy.logwarn('Started!')

    # when client want to update task_list, cancel the current jobs and update the
    def update_task_list(self, req):
        task_list = np.array(req.list.data).reshape((-1, 10))
        rospy.loginfo('received new task_list: ')
        rospy.loginfo(task_list)
        self.task_list = task_list.copy()  # copy a new duplicate, do not assign the reference

        self.path_plan_client.cancel_all_goals()

        # rospy.sleep(0.5)
        self.stop()

        return True

    def execute_task(self):
        if self.task_list is not None:
            self.no_more_task_warned = 0

            task_num = self.task_list.shape[0]
            task_list_cur = self.task_list[0, :].copy()
            rospy.logwarn('Executing task: ')
            rospy.logwarn(task_list_cur)

            if task_num == 1:
                self.task_list = None
            else:
                self.task_list = self.task_list[1:task_num, :].copy()

            if task_list_cur[0] == 0:  # stop mode, [0, ...]
                self.path_plan_client.cancel_all_goals()
                # publish all zero velocity cmd
                self.stop()
            elif task_list_cur[0] == 1:  # path plan mode, [1, x, y, theta, ...]
                goal_pose = task_list_cur[1:4].copy()
                self.path_plan_action(goal_pose)
            else:
                rospy.logerr('unknown task code.')
        else:
            if not self.no_more_task_warned:
                rospy.logwarn('Task list empty now.')
                self.no_more_task_warned = 1

    # move_dir:
    # 1. move_base task: 0, stay still; 1, move forward; 2, move backward; 3, move left; 4, move right
    # 5, move forward no jump
    # 9, turn 180
    def path_plan_action(self, move_dir):
        goal = cashmerebot.msg.path_planGoal()
        goal.target_location = [int(move_dir)]

        self.path_plan_client.send_goal(goal)
        rospy.logerr('line_track_client: sent new goal (%f)' % (goal.target_location[0]))

        self.path_plan_client.wait_for_result()
        rospy.logerr("line_track_client: goal completed")

    def stop(self):
        pass


def main(args):
    rospy.init_node('task_manager_node', anonymous=True)
    tm = TaskManager()
    try:
        while not rospy.is_shutdown():
            tm.execute_task()
            rospy.sleep(0.2)
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)