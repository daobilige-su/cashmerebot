#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import cashmerebot.msg
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int8MultiArray
import cv2
from cv_bridge import CvBridge, CvBridgeError
import datetime
import time

# params

class path_plan_action(object):
    _feedback = cashmerebot.msg.path_planFeedback()
    _result = cashmerebot.msg.path_planResult()

    def __init__(self):
        self._action_name = 'path_plan'
        self._as = actionlib.SimpleActionServer(self._action_name, cashmerebot.msg.path_planAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal): # 0: stand still, 1: forward; 2, backward; 3, left; 4, right;

        success = False
        params = goal.params

        rospy.loginfo('%s: Executing, obtained goal location: (%f)' % (
            self._action_name, goal.params[0]))

        # do something here
        success = True

        if success == True:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':

    rospy.init_node('path_plan_server_node')
    server = path_plan_action()
    rospy.spin()
