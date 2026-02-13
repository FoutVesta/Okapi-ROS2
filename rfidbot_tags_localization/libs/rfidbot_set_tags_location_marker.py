#!/usr/bin/env python3
'''
* File name: rfidbot_set_tags_location_marker.py
* Description: provide the function of setting a marker
* Author: Jian Zhang
* Create date: June/17/2015
* Modified date: Nov/10/2025 by Justin Palm
* Version 2.0 for ROS2 Humble
'''

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time


# Helper node to access ROS2 time for header.stamp
_node_instance = None
def _get_node():
    global _node_instance
    if _node_instance is None:
        # Avoid calling rclpy.init() twice; reuse existing context if already initialized.
        if not rclpy.ok():
            rclpy.init(args=None)
        _node_instance = Node('marker_time_helper')
    return _node_instance


def setaNewMarker(PosX, PosY, OriZ, OriW, color, size, frameId):
    marker = Marker()
    marker.header.frame_id = frameId
    marker.header.stamp = _get_node().get_clock().now().to_msg()
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale.x = size * 45
    marker.scale.y = size * 30
    marker.scale.z = size
    if color == 'r':
        marker.color.a = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    elif color == 'g':
        marker.color.a = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif color == 'y':
        marker.color.a = 0.05
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif color == 'b':
        marker.color.a = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
    marker.pose.orientation.z = OriZ
    marker.pose.orientation.w = OriW
    marker.pose.position.x = PosX
    marker.pose.position.y = PosY
    return marker


def setaSingle3DMarker(PosX, PosY, PosZ, color, size, frameId):
    marker = Marker()
    marker.header.frame_id = frameId
    marker.header.stamp = _get_node().get_clock().now().to_msg()
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    if color == 'r':
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    elif color == 'g':
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif color == 'b':
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = PosX
    marker.pose.position.y = PosY
    marker.pose.position.z = PosZ
    return marker


def setaSingleMarkerText(PosX, PosY, PosZ, tagEpc, frameId):
    marker = Marker()
    marker.header.frame_id = frameId
    marker.header.stamp = _get_node().get_clock().now().to_msg()
    marker.ns = "all_scanned_tags_text"
    marker.id = 0
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD

    marker.pose.position.x = PosX
    marker.pose.position.y = PosY - 0.2
    marker.pose.position.z = PosZ

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    marker.text = tagEpc[-4:]
    return marker


def setaNewArrowMarker(PosX, PosY, OriZ, OriW, color, size_x, size_y, size_z, frameId):
    marker = Marker()
    marker.header.frame_id = frameId
    marker.header.stamp = _get_node().get_clock().now().to_msg()
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale.x = size_x
    marker.scale.y = size_y
    marker.scale.z = size_z
    if color == 'r':
        marker.color.a = 0.65
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    elif color == 'g':
        marker.color.a = 0.65
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif color == 'y':
        marker.color.a = 0.65
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif color == 'b':
        marker.color.a = 0.65
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
    marker.pose.orientation.z = OriZ
    marker.pose.orientation.w = OriW
    marker.pose.position.x = PosX
    marker.pose.position.y = PosY
    return marker


def setaSingleMarker(PosX, PosY, color, size, frameId):
    marker = Marker()
    marker.header.frame_id = frameId
    marker.header.stamp = _get_node().get_clock().now().to_msg()
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    if color == 'r':
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    elif color == 'g':
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif color == 'y':
        marker.color.a = 0.05
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif color == 'b':
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = PosX
    marker.pose.position.y = PosY
    marker.pose.position.z = 0
    return marker


def setMarkerwithTransparent(PosX, PosY, color, size, trans, frameId):
    marker = Marker()
    marker.header.frame_id = frameId
    marker.header.stamp = _get_node().get_clock().now().to_msg()
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    if color == 'r':
        marker.color.a = trans
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    elif color == 'g':
        marker.color.a = trans
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif color == 'y':
        marker.color.a = trans
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif color == 'b':
        marker.color.a = trans
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = PosX
    marker.pose.position.y = PosY
    marker.pose.position.z = 0
    return marker
