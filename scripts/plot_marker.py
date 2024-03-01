#!/usr/bin/env python
# -*- coding: utf-8 -*-

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from visualization_msgs.msg import Marker


def plot_pts(pts, marker_pub, frame_id='map'):
    marker_array = MarkerArray()
    marker_array.markers = []

    # pts
    pt_marker = Marker()
    pt_marker.header.frame_id = frame_id
    pt_marker.ns = "path_plan_" + "pt"
    pt_marker.id = 0
    pt_marker.type = Marker.CUBE_LIST
    pt_marker.action = Marker.ADD
    pose = Pose()
    pose.orientation.w = 1.0
    pt_marker.pose = pose
    # when list is used, color needs to be 1.0 not 255, such a bug!
    pt_marker.color.r, pt_marker.color.g, pt_marker.color.b = (1.0, 1.0, 0)
    pt_marker.color.a = 1.0
    pt_marker.scale.x, pt_marker.scale.y, pt_marker.scale.z = (0.01, 0.01, 0.01)

    pt_marker.points = []
    # pt_marker.colors = []
    pts_num = pts.shape[0]
    for i in range(pts_num):
        pt = Point()
        pt.x = pts[i, 0]
        pt.y = pts[i, 1]
        pt.z = pts[i, 2]
        pt_marker.points.append(pt)

        # color = ColorRGBA()
        # color.r, color.g, color.b = (255, 255, 255)
        # color.a = 1.0
        # pt_marker.colors.append(color)

    marker_array.markers.append(pt_marker)

    # publish marker_array
    marker_pub.publish(marker_array)


def plot_traj(traj_pts, marker_pub, frame_id='map'):
    marker_array = MarkerArray()
    marker_array.markers = []

    # traj_pts
    pt_marker = Marker()
    pt_marker.header.frame_id = frame_id
    pt_marker.ns = "path_plan_" + "traj_pts"
    pt_marker.id = 1
    pt_marker.type = Marker.CUBE_LIST
    pt_marker.action = Marker.ADD
    pose = Pose()
    pose.orientation.w = 1.0
    pt_marker.pose = pose
    # when list is used, color needs to be 1.0 not 255, such a bug!
    pt_marker.color.r, pt_marker.color.g, pt_marker.color.b = (1.0, 0.0, 0)
    pt_marker.color.a = 1.0
    pt_marker.scale.x, pt_marker.scale.y, pt_marker.scale.z = (0.01, 0.01, 0.01)

    pt_marker.points = []
    # pt_marker.colors = []
    traj_pts_num = traj_pts.shape[0]
    for i in range(traj_pts_num):
        pt = Point()
        pt.x = traj_pts[i, 0]
        pt.y = traj_pts[i, 1]
        pt.z = traj_pts[i, 2]
        pt_marker.points.append(pt)

        # color = ColorRGBA()
        # color.r, color.g, color.b = (255, 255, 255)
        # color.a = 1.0
        # pt_marker.colors.append(color)

    marker_array.markers.append(pt_marker)

    # traj line
    line_marker = Marker()
    line_marker.header.frame_id = "map"
    line_marker.ns = "path_plan_" + "traj_line"
    line_marker.id = 0
    line_marker.type = Marker.LINE_STRIP
    line_marker.action = Marker.ADD
    pose = Pose()
    pose.orientation.w = 1
    line_marker.pose = pose
    line_marker.color.r, line_marker.color.g, line_marker.color.b = (1.0, 0.0, 0.0)
    line_marker.color.a = 0.5
    line_marker.scale.x, line_marker.scale.y, line_marker.scale.z = (0.1, 0.1, 0.1)

    line_marker.points = []
    for i in range(traj_pts_num):
        pt = Point()
        pt.x = traj_pts[i, 0]
        pt.y = traj_pts[i, 1]
        pt.z = traj_pts[i, 2]
        line_marker.points.append(pt)

    marker_array.markers.append(line_marker)

    # publish marker_array
    marker_pub.publish(marker_array)