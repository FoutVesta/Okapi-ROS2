#!/usr/bin/env python3
'''
*File name: rfh_tags_localization_fix_power_rawdata_recorder.py
*Description: record the tag localization raw data and combine with reader pose
              the reader is working under fixed power
*Author: Jian Zhang
*Create date: Aug/10/2016
*Modified date: Nov/10/2025 by Justin Palm
*Version 2.0 for ROS2 Humble
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import sys, os, os.path, pickle

# ROS 2 Python packages
from rfidbot_tags_interfaces.msg import TagReader

# Import internal modules within package namespace
from rfidbot_tags_localization.data_recorders.rfidbot_tags_localization_readrate_rawdata_base import rfidbotTagsLocReadRateRawData
from rfidbot_tags_localization.data_recorders.rfidbot_tags_localization_rawdata_recorder_base import rfidbotTagLocRawDataRecordBase

from rfh_share_lib.rfidbot_set_para_2_reader import rfidbotReaderFilterSetter
from rfidbot_tags_localization.libs.rfh_tags_localization_pose_shifter import rfhposeShifter


class rfhFixPowerRecoder(rfidbotTagLocRawDataRecordBase):
    def __init__(self, node, rate_ratio, rate, odom_topic="/odom", rfid_topic="/rfid_tags"):
        super().__init__(node, odom_topic=odom_topic, rfid_topic=rfid_topic)

        self.name = "rfh fix power raw data recorder"
        self.rate_ratio = rate_ratio
        self.rate_hz = rate
        self.rate = self.node.create_rate(rate)
        self.tagLocRawData = []

        # Parameters (declare + get)
        self.node.declare_parameter('txpower', 130)
        self.node.declare_parameter('antenna_frameid', "RFD8500_antenna")
        self.map_frameid = self.node.get_parameter('map_frameid').value

        self.power = int(self.node.get_parameter('txpower').value)
        self.antennaFrameId = self.node.get_parameter('antenna_frameid').value
        self.mapFrameId = self.node.get_parameter('map_frameid').value

        self.poseShifter = rfhposeShifter(self.node)

        # Initialize filter setter for power reset and tag filter reset
        self.isResetPower = False
        self.filterSetter = rfidbotReaderFilterSetter(self.node, self.rate_ratio, self.rate_hz)

        # ROS 2 subscription
        self.node.create_subscription(Bool, '/set_tx', self.setPowerCallback, 10)

        # Initial setup delay and reset
        self.waitForPeriod(2.5)
        self.resetPowerLevel2Reader()

    def setPowerCallback(self, msg):
        if msg.data:
            self.resetPowerLevel2Reader()

    def resetPowerLevel2Reader(self):
        self.node.get_logger().warn(f"Setting power level {self.power}")
        self.filterSetter.pauseInven2DelRos()
        self.waitForPeriod(1.5)
        self.filterSetter.setTxPower(self.power)
        self.waitForPeriod(1.5)
        self.filterSetter.resumeInvenWithNewRos()
        self.isResetPower = True

    def waitForPeriod(self, idle_seconds):
        if idle_seconds is None:
            return
        for _ in range(int(self.rate_ratio * idle_seconds)):
            self.rate.sleep()

    def rfidtagsCallBack(self, msg):
        if not self.isResetPower:
            self.node.get_logger().warn("RFID tag received before power reset; ignoring.")
            return
        msg.epc = msg.epc.lower()
        tag = msg
        candidatePose = self.currentPose
        if candidatePose is None:
            self.node.get_logger().warn("No current pose; recording tag with empty pose.")
        tmpRawData = self.initARawDataByTagandPose(tag, candidatePose)
        self.tagLocRawData.append(tmpRawData)
        has_pose = tmpRawData.antennaPose is not None
        self.node.get_logger().info(
            f"Recorded tag {tag.epc}, total raw entries: {len(self.tagLocRawData)}, with pose: {has_pose}"
        )

    def getUniqueTags(self):
        for _, rawData in enumerate(self.tagLocRawData):
            if rawData.TagEpc not in self.uniqueTagsEPC:
                self.uniqueTagsEPC.append(rawData.TagEpc)
        self.node.get_logger().warn(f"Total scanned unique tag number: {len(self.uniqueTagsEPC)}")

    def saveRawData2File(self, rawDataFile=None):
        if rawDataFile is None:
            rawDataFile = self.rawDataFileAddr
        with open(rawDataFile, 'wb') as f:
            pickle.dump(self.tagLocRawData, f)
        self.node.get_logger().warn(f"Saved raw data to file {rawDataFile}")

    def readRawDataFromFile(self, rawDataFile=None):
        if rawDataFile is None:
            rawDataFile = self.rawDataFileAddr
        if not os.path.isfile(rawDataFile):
            self.node.get_logger().warn(f"Raw data file {rawDataFile} does not exist")
            return
        with open(rawDataFile, 'rb') as f:
            self.tagLocRawData = pickle.load(f)
        self.node.get_logger().warn(f"Read raw data from file {rawDataFile}")
        self.node.get_logger().warn(f"Size of raw data: {len(self.tagLocRawData)}")

    def initARawDataByTagandPose(self, tag, candidatePose):
        tmpRawData = rfidbotTagsLocReadRateRawData()
        tmpRawData.TagEpc = tag.epc
        tmpRawData.antennaID = tag.antenna_id
        tmpRawData.antennaPose = self.tfCameraPose2AntennaPose(candidatePose, tmpRawData.antennaID)
        tmpRawData.readRate = 1
        tmpRawData.powerLevel = self.power
        tmpRawData.phase = tag.phase
        tmpRawData.channelID = tag.channel
        tmpRawData.peakRSSI = tag.peak_rssi
        return tmpRawData

    def tfCameraPose2AntennaPose(self, candidatePose, antennaID):
        if candidatePose is None:
            return None
        if antennaID == "RFD8500_1":
            targetFrame = self.antennaFrameId
        else:
            targetFrame = f"RFID_antenna_{antennaID}"
        sourceframe = candidatePose.child_frame_id
        globalFrame = self.mapFrameId
        newpose = Odometry()
        newpose.pose = self.poseShifter.shiftPose(sourceframe, targetFrame, globalFrame, candidatePose)
        return newpose


def main(args=None):
    rclpy.init(args=args)
    node = rfhFixPowerRecoder(rate_ratio=10, rate_hz=10.0)  # Example defaults
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
