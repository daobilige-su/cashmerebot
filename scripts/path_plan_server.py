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

        self.surf_dist = 0.1
        self.angle_incr = 5  # deg

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

        # extract mid line in 2 points
        mid_line = np.array([[-0.9, 0.5],[0.5, 0.5], [0.3, 0.3]])


        # path plan
        cloud_ring = cloud_seg.copy()
        cloud_ring = cloud_ring[:, cloud_ring[0, :] > -0.05]
        cloud_ring = cloud_ring[:, cloud_ring[0, :] < 0.05]

        plot_pts(cloud_ring.T, self.marker_pub, 'ur5_base')
        plot_traj(mid_line.T, self.marker_pub, 'ur5_base')

        ring_center = mid_line[:, 0]
        ring_center[0] = 0
        ring_center = ring_center.reshape((-1,))
        can_pose_ypr, can_pts_num = self.plan_single_ring(cloud_ring, ring_center)

        plot_traj(can_pose_ypr[0:3, :].T, self.marker_pub, 'ur5_base')






        success = True

        if success == True:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

    def plan_single_ring(self, cloud, center):
        cloud_2d = cloud[1:3, :]
        center_2d = center[1:3]

        angle = np.arctan2(cloud_2d[1, :]-center_2d[1], -(cloud_2d[0, :]-center_2d[0]))
        angle_deg = np.rad2deg(angle)

        can_deg = range(-90, 90+self.angle_incr, self.angle_incr)
        can_num = len(can_deg)
        can_pts_num = np.zeros((1, can_num))
        can_pose_ypr = np.zeros((6, can_num))

        for idx in range(can_num):
            deg = can_deg[idx]

            deg_cloud_idx = (abs(angle_deg-deg)<(self.angle_incr/2.0))
            deg_cloud_pts = cloud_2d[:, deg_cloud_idx]

            deg_cloud_pts_num = deg_cloud_pts.shape[1]
            can_pts_num[0, idx] = deg_cloud_pts_num

            if deg_cloud_pts_num<5:
                continue

            deg_cloud_pts_mean = np.mean(deg_cloud_pts, 1)
            dist_mean = np.sqrt(np.sum(np.power(center_2d - deg_cloud_pts_mean, 2)))
            dist_arm = dist_mean+self.surf_dist

            deg_cloud_pose = np.array([0, center_2d[0]-dist_arm*np.cos(np.deg2rad(deg)), center_2d[1]+dist_arm*np.sin(np.deg2rad(deg)), \
                                       np.pi/2, np.deg2rad(deg), 0]).reshape((6,1))
            can_pose_ypr[:,idx:idx+1] = deg_cloud_pose

        return can_pose_ypr, can_pts_num











if __name__ == '__main__':

    rospy.init_node('path_plan_server_node')
    server = path_plan_action()
    rospy.spin()
