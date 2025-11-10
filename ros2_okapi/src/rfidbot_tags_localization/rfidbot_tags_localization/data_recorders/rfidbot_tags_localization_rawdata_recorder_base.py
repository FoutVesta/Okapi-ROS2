#!/usr/bin/env python3
'''
*File name: rfidbot_tags_localization_rawdata_recorder_base.py
*Description: provides the basic data structure of tag localization rawdata recorder
*Author: Jian Zhang
*Create date: Feb/15/2016
*Modified date: Nov/10/2025 by Justin Palm
*Version 2.0 for ROS2 Humble
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from rfidbot_tags_reader.msg import TagReader
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import pickle
import os

FLOAT_ZERO = 1e-8
BAD_POSITION = 9999.9


class rfidbotTagLocRawDataRecordBase(Node):
    """
    Base class for RFID tag localization raw data recorder (ROS 2)
    """

    def __init__(self):
        super().__init__('rfidbot_tag_loc_rawdata_recorder_base')

        self.tags = []
        self.uniqueTagsEPC = []     # unique tag EPCs; call getUniqueTags() to populate
        self.postions = []
        self.currentPose = None
        self.tagLocRawData = None
        self.rawDataFileAddr = None

        # ROS 2 subscriptions
        self.create_subscription(TagReader, '/rfid_tags', self.rfidtagsCallBack, 10)
        self.create_subscription(Odometry, '/odom', self.poseCallback, 10)
        # self.create_subscription(PoseWithCovarianceStamped, '/pose', self.poseCallback, 10)

    # Simply append tag to array
    def rfidtagsCallBack(self, msg):
        msg.EPC = msg.EPC.lower()
        self.tags.append(msg)
        self.get_logger().warn(f"tag {len(self.tags)}")

    def getUniqueTags(self):
        """
        From all recorded RFID tags, generate a unique tag list.
        """
        self.get_logger().warn(f"tag number: {len(self.tags)}")
        for tag in self.tags:
            self.get_logger().warn(f"tag {tag.EPC}")
            if tag.EPC not in self.uniqueTagsEPC:
                self.uniqueTagsEPC.append(tag.EPC)
        self.get_logger().warn(f"Total scanned unique tag number: {len(self.uniqueTagsEPC)}")

    def isPoseValid(self, pose):
        if (
            pose.pose.pose.position.x == BAD_POSITION and
            pose.pose.pose.position.y == BAD_POSITION and
            pose.pose.pose.position.z == BAD_POSITION and
            pose.pose.pose.orientation.x == BAD_POSITION and
            pose.pose.pose.orientation.y == BAD_POSITION and
            pose.pose.pose.orientation.z == BAD_POSITION and
            pose.pose.pose.orientation.w == BAD_POSITION
        ):
            return False
        else:
            return True

    def poseCallback(self, msg):
        if not self.isPoseValid(msg):
            self.currentPose = None
            # self.get_logger().warn("get pose is Null")
            return
        self.currentPose = msg
        # self.get_logger().warn(f"get pose {self.currentPose.pose.pose.position.x}")
        self.postions.append(msg)

    def saveRawData2File(self):
        if self.rawDataFileAddr is None:
            self.get_logger().warn('the file address is None!')
            return

        with open(self.rawDataFileAddr, 'wb') as f:
            pickle.dump(self.tagLocRawData, f)
        self.get_logger().warn(f"recorder: save raw data to file {self.rawDataFileAddr}")

    def readRawDataFromFile(self):
        if self.rawDataFileAddr is None:
            self.get_logger().warn('the file address is None!')
            return

        with open(self.rawDataFileAddr, 'rb') as f:
            self.tagLocRawData = pickle.load(f)

        recordLength = len(self.tagLocRawData) if self.tagLocRawData is not None else 0
        self.get_logger().warn(
            f"recorder: read raw data from file {self.rawDataFileAddr}, with length {recordLength}"
        )

    def setRawDataFileAdr(self, fileAddr):
        self.rawDataFileAddr = fileAddr

    def getRawDataForATag(self, TagEpc):
        """
        Return all raw data entries for a specific tag EPC.
        """
        RawData = []
        if self.tagLocRawData is None:
            return None

        for _, rawData in enumerate(self.tagLocRawData):
            if TagEpc == rawData.TagEpc:
                RawData.append(rawData)

        if len(RawData) < 1:
            return None

        return RawData


def main(args=None):
    rclpy.init(args=args)
    node = rfidbotTagLocRawDataRecordBase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

