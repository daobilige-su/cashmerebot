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
import ros_numpy
from plot_marker import *

# params


class manipulation_action(object):
    _feedback = cashmerebot.msg.manipulationFeedback()
    _result = cashmerebot.msg.manipulationResult()

    def __init__(self):
        self._action_name = 'manipulation'
        self._as = actionlib.SimpleActionServer(self._action_name, cashmerebot.msg.manipulationAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # params:

    def execute_cb(self, goal): # 0: stand still, 1: forward; 2, backward; 3, left; 4, right;

        success = False
        path_np = np.array(goal.path).reshape((-1, 6)).T

        rospy.loginfo('%s: obtained path, executing ...' % (self._action_name))

        # return result
        success = True

        if success == True:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':

    rospy.init_node('manipulation_server_node')
    server = manipulation_action()
    rospy.spin()
