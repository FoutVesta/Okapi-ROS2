#!/usr/bin/env python3
'''
* File name: rfidbot_tags_recursive_bayes_localize.py
* Description: Based on raw tag reading location, use a probabilistic approach
*              (recursive Bayesian updating) to localize a tag.
* Author: Jian Zhang
* Original date: Sep/29/2015
* Modified Date: Nov/10/2025 by Justin Palm
* Version 2.0 for ROS2 Humble
'''

import math
import numpy as np
import cv2
from math import pi
from matplotlib import pyplot as plt
from PIL import Image

import sys, os
import rclpy
from rclpy.logging import get_logger

# Local imports (ROS2-style)
from rfidbot_tags_localization.localizers.rfidbot_tags_antenna_model import rfidbotRFIDModel
from rfidbot_tags_localization.data_recorders.rfidbot_tags_localization_readrate_rawdata_base import (
    rfidbotTagsLocReadRateRawData,
)

antennaMaxRange = 3.4  # 3.2


class rfidbotRBLocalizeATag:
    """
    Class: rfidbotRBLocalizeATag
    Uses recursive Bayesian updating to localize a tag in 2D.
    """

    def __init__(self):
        self.logger = get_logger("rfidbotRBLocalizeATag")

        self.posofEPC = None  # [tag.AntennaID, newPose(antenna's pose)]
        self.EPC = None
        self.estimatedLocation = None  # estimated tag locations (xt)
        self.esLocBel = None  # [{'esloc': xx, 'bel': xx}]
        self.RFIDModel = rfidbotRFIDModel()
        self.setRFIDModel()

        # For debug
        self.antennaBelInmap = None
        self.rawData = None  # [rfidbotTagsLocReadRateRawData]

    # ----------------------------------------------------------
    # MODEL SETUP
    # ----------------------------------------------------------
    def setRFIDModel(self):
        self.RFIDModel.AntennaInfo = ""
        self.RFIDModel.BeliefMapResolution = 0.05  # map resolution in meters
        self.RFIDModel.generateRFIDModelMap()
        self.logger.info("RFID model initialized with resolution %.2f",
                         self.RFIDModel.BeliefMapResolution)

    # ----------------------------------------------------------
    # DATA SETUP
    # ----------------------------------------------------------
    def setANewEPCAndPos(self, EPC, rawData):
        self.EPC = EPC
        self.rawData = rawData

    def getAntennaAngle(self, pose):
        """Extract yaw (in degrees) from a quaternion pose."""
        qx = pose.pose.pose.orientation.x
        qy = pose.pose.pose.orientation.y
        qz = pose.pose.pose.orientation.z
        qw = pose.pose.pose.orientation.w
        degree = math.atan2(2 * qx * qy + 2 * qw * qz,
                            qw * qw + qx * qx - qy * qy - qz * qz)
        return (degree * 180.0) / math.pi

    # ----------------------------------------------------------
    # INITIALIZATION
    # ----------------------------------------------------------
    def initEstimatedLoc(self):
        """Initialize the first estimated tag location from the first reading."""
        initLocSet = []
        initAntennaPose = self.posofEPC[0][1]
        rotDegree = self.getAntennaAngle(initAntennaPose)
        NewModel = self.RFIDModel.rotateModel(rotDegree)

        offsetX = initAntennaPose.pose.pose.position.x - self.RFIDModel.AntennaLoc[1]
        offsetY = initAntennaPose.pose.pose.position.y - self.RFIDModel.AntennaLoc[0]

        for i in range(self.RFIDModel.mapHeight):
            for j in range(self.RFIDModel.mapWidth):
                if NewModel[i][j] > 0:
                    initEstX = j * self.RFIDModel.BeliefMapResolution + offsetX
                    initEstY = i * self.RFIDModel.BeliefMapResolution + offsetY
                    initLocSet.append([initEstX, initEstY])
        return initLocSet

    def initEstimatedPosMaxArea(self):
        """Initialize estimated positions using bounding box of antenna poses."""
        initLocSet = []
        MinX = self.rawData[0].antennaPose.pose.pose.position.x
        MinY = self.rawData[0].antennaPose.pose.pose.position.y
        MaxX = MinX
        MaxY = MinY

        for rawItem in self.rawData:
            posX = rawItem.antennaPose.pose.pose.position.x
            posY = rawItem.antennaPose.pose.pose.position.y
            MinX, MaxX = min(MinX, posX), max(MaxX, posX)
            MinY, MaxY = min(MinY, posY), max(MaxY, posY)

        MinX -= antennaMaxRange
        MinY -= antennaMaxRange
        MaxX += antennaMaxRange
        MaxY += antennaMaxRange

        x = MinX
        while x < MaxX:
            y = MinY
            while y < MaxY:
                initLocSet.append([x, y])
                y += self.RFIDModel.BeliefMapResolution
            x += self.RFIDModel.BeliefMapResolution
        return initLocSet

    def initEsLocBel(self):
        """Initialize belief list for each hypothesis location."""
        self.esLocBel = [{'esloc': loc, 'bel': 1.0, 'chBel': [0] * 50}
                         for loc in self.estimatedLocation]
        self.antennaBelInmap = [{'esloc': loc, 'bel': 1.0}
                                for loc in self.estimatedLocation]

    # ----------------------------------------------------------
    # BAYESIAN MEASUREMENT MODEL
    # ----------------------------------------------------------
    def getMeasurement(self, posX, posY, posTh, esLocX, esLocY):
        """Compute measurement likelihood for estimated position."""
        Th = posTh / 180.0 * math.pi
        esLox_1 = esLocX * math.cos(Th) + esLocY * math.sin(Th)
        esLoy_1 = esLocY * math.cos(Th) - esLocX * math.sin(Th)
        anPosx_1 = posX * math.cos(Th) + posY * math.sin(Th)
        anPosy_1 = posY * math.cos(Th) - posX * math.sin(Th)

        newPosX = esLox_1 - (anPosx_1 - self.RFIDModel.AntennaLoc[0])
        newPosY = (anPosy_1 + self.RFIDModel.AntennaLoc[1]) - esLoy_1

        mapIdCol = int(round(newPosX / self.RFIDModel.BeliefMapResolution))
        mapIdRow = int(round(newPosY / self.RFIDModel.BeliefMapResolution))

        if (mapIdCol < 0 or mapIdCol >= self.RFIDModel.mapWidth or
                mapIdRow < 0 or mapIdRow >= self.RFIDModel.mapHeight):
            return self.RFIDModel.zeroBel

        measurement = self.RFIDModel.BeliefMap[mapIdRow][mapIdCol]
        if measurement == 0:
            measurement = self.RFIDModel.zeroBel
        return measurement

    def belUpdate(self, idx, esLoc, pose):
        """Update the belief for one estimated position."""
        posX = pose.pose.pose.position.x
        posY = pose.pose.pose.position.y
        posTh = self.getAntennaAngle(pose)
        meas = self.getMeasurement(posX, posY, posTh, esLoc[0], esLoc[1])
        newBel = self.esLocBel[idx]['bel'] * meas
        self.esLocBel[idx]['bel'] = newBel
        return newBel

    # ----------------------------------------------------------
    # RAW DATA HANDLING
    # ----------------------------------------------------------
    def minDistance2(self, newRawData, rawItem):
        """Find squared min distance between a rawItem and list of rawData."""
        posX = rawItem.antennaPose.pose.pose.position.x
        posY = rawItem.antennaPose.pose.pose.position.y
        minDis2 = self.RFIDModel.calDistance(
            posX, posY,
            newRawData[0].antennaPose.pose.pose.position.x,
            newRawData[0].antennaPose.pose.pose.position.y,
        )
        for item in newRawData:
            tmpX = item.antennaPose.pose.pose.position.x
            tmpY = item.antennaPose.pose.pose.position.y
            dis2 = self.RFIDModel.calDistance(posX, posY, tmpX, tmpY)
            if dis2 < minDis2:
                minDis2 = dis2
        return minDis2

    def rawDataFilter(self):
        """Filter raw data: remove readings too close to existing ones."""
        if self.rawData is None or len(self.rawData) < 1:
            return
        newRawData = [self.rawData[0]]
        thres2 = (self.RFIDModel.BeliefMapResolution * 0.5) ** 2
        for rawItem in self.rawData:
            minDis2 = self.minDistance2(newRawData, rawItem)
            if minDis2 > thres2:
                newRawData.append(rawItem)
        self.rawData = newRawData

    # ----------------------------------------------------------
    # LOCALIZATION PROCESS
    # ----------------------------------------------------------
    def localizesATag(self):
        """Main localization loop: performs recursive Bayesian update."""
        if self.rawData is None or len(self.rawData) < 1:
            self.logger.warn("Input rawData is None or empty.")
            return None

        # 1. Filter data
        self.rawDataFilter()

        # 2. Initialize positions and beliefs
        self.estimatedLocation = self.initEstimatedPosMaxArea()
        self.initEsLocBel()

        self.logger.info("Starting Bayesian updates...")
        for rawItem in self.rawData:
            normalizer = 0.0
            for idx, esLoc in enumerate(self.estimatedLocation):
                newBel = self.belUpdate(idx, esLoc, rawItem.antennaPose)
                normalizer += newBel

            # Normalize all beliefs
            if normalizer != 0:
                for entry in self.esLocBel:
                    entry['bel'] /= normalizer

        # 3. Find the maximum belief (most likely tag location)
        maxBel = 0
        maxLoc = None
        for entry in self.esLocBel:
            if entry['bel'] > maxBel:
                maxBel = entry['bel']
                maxLoc = entry['esloc']

        self.logger.info("Localization complete. Max belief = %.4f", maxBel)
        return maxLoc


# ----------------------------------------------------------
# TEST / STANDALONE RUN
# ----------------------------------------------------------
if __name__ == "__main__":
    rclpy.init()
    loc = rfidbotRBLocalizeATag()
    loc.logger.info("Testing RFID Bayesian localization base class...")
    loc.RFIDModel.BeliefMapResolution = 0.05
    loc.RFIDModel.generateRFIDModelMap()
    loc.logger.info("Belief map shape: %s x %s", loc.RFIDModel.mapWidth, loc.RFIDModel.mapHeight)
    rclpy.shutdown()

