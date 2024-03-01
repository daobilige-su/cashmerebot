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

class path_plan_action(object):
    _feedback = cashmerebot.msg.path_planFeedback()
    _result = cashmerebot.msg.path_planResult()

    def __init__(self):
        self._action_name = 'path_plan'
        self._as = actionlib.SimpleActionServer(self._action_name, cashmerebot.msg.path_planAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # params:
        self.pc_roi = np.array([[-1, 1], [0.2, 0.8], [0, 1]])
        self.marker_pub = rospy.Publisher("path_plan_markers", MarkerArray, queue_size=5)

    def execute_cb(self, goal): # 0: stand still, 1: forward; 2, backward; 3, left; 4, right;

        success = False
        params = goal.params

        rospy.loginfo('%s: Executing, obtained goal location: (%f)' % (
            self._action_name, goal.params[0]))

        # read point cloud
        pc2_msg = rospy.wait_for_message('/front_depth_cam/pc2_ur5_base', PointCloud2, timeout=1.0)
        cloud = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg).T  # 3xN

        # ROI
        cloud_roi = cloud.copy()
        # X limit
        cloud_roi = cloud_roi[:, cloud_roi[0, :] > self.pc_roi[0, 0]]
        cloud_roi = cloud_roi[:, cloud_roi[0, :] < self.pc_roi[0, 1]]
        # Y limit
        cloud_roi = cloud_roi[:, cloud_roi[1, :] > self.pc_roi[1, 0]]
        cloud_roi = cloud_roi[:, cloud_roi[1, :] < self.pc_roi[1, 1]]
        # Z limit
        cloud_roi = cloud_roi[:, cloud_roi[2, :] > self.pc_roi[2, 0]]
        cloud_roi = cloud_roi[:, cloud_roi[2, :] < self.pc_roi[2, 1]]

        # segment
        cloud_seg = cloud_roi.copy()

        # extract mid line


        # path plan
        cloud_ring = cloud_seg.copy()
        cloud_ring = cloud_ring[:, cloud_ring[0, :] > -0.05]
        cloud_ring = cloud_ring[:, cloud_ring[0, :] < 0.05]

        plot_pts(cloud_ring.T, self.marker_pub, 'ur5_base')



        success = True

        if success == True:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':

    rospy.init_node('path_plan_server_node')
    server = path_plan_action()
    rospy.spin()
