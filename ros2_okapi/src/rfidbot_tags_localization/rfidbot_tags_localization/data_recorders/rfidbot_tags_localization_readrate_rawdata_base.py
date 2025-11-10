#!/usr/bin/env python3
'''
*File name: rfidbot_tags_localization_readrate_rawdata_base.py
*Description: provide the data class for raw data of read rate recording
*Author: Jian Zhang
*Create date: Oct/30/2015
*Modified date: Nov/10/2025
*Version 2.0 for ROS2 Humble
'''

# No ROS runtime dependencies needed here
# This module only defines plain Python data structures

class rfidbotTagsLocReadRateRawData:
    """
    Data class for a single RFID tag read-rate record.
    Designed for cases where the robot stops at a position to read tags.
    Each tag with the same antenna ID during a reading period
    will occupy one object of this class.
    """

    def __init__(self):
        self.TagEpc = None
        self.antennaID = None
        self.antennaPose = None      # antenna position (geometry_msgs/Pose)
        self.readRate = None
        self.readAmount = None
        self.firstReadTimeStamp = None
        self.lastReadTimeStamp = None
        self.readingPeriod = None
        self.environmentSacnnedTagsNum = None
        self.powerLevel = None       # e.g., 80, 110...
        self.phase = None            # string
        self.channelID = None        # string
        self.peakRSSI = None         # string

    def updateReadRate(self):
        """Compute read rate from amount / period."""
        if self.readingPeriod and self.readingPeriod != 0:
            self.readRate = self.readAmount / self.readingPeriod


class rfidbotTagsLocReadRateVectorRawData:
    """
    Contains all power-level read-rate values recorded
    at the same position and antenna.
    """

    def __init__(self):
        self.TagEpc = None
        self.antennaID = None
        self.antennaPose = None      # antenna position
        self.powerLevel = None       # list, e.g., [80, 110, ...]
        self.readRateVector = None   # list, e.g., [rr80, rr110, ...]

