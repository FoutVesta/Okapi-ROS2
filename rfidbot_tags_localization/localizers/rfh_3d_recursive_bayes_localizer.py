#!/usr/bin/env python3
'''
* File name: rfh_3d_recursive_bayes_localizer.py
* Description: based on the RFusion handheld raw tag reading location in 3D,
*              use a probabilistic approach based on recursive Bayesian updating
*              to localize a tag in 3D.
* Author: Jian Zhang
* Modified by Justin Palm Nov/10/2025
* Version 2.0 for ROS2 Humble
'''

import os
import pickle
import time
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import importlib
import sys
import types

from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped
import tf_transformations  # replaces `tf.transformations`
import tf2_ros
import tf2_geometry_msgs


# ---- Absolute imports within your ROS2 package ----
from rfidbot_tags_localization.localizers.rfidbot_tags_recursive_bayes_localize import rfidbotRBLocalizeATag
from rfidbot_tags_localization.libs.rfh_tags_localization_pose_shifter import rfhposeShifter
from rfidbot_tags_localization.libs.rfidbot_tags_debuger import RRRBDebuger

# Paths
module_dir = os.path.dirname(os.path.abspath(__file__))
pardir = os.path.abspath(os.path.join(module_dir, os.pardir))
# Source tree root (one level above the Python package folder)
root_dir = os.path.abspath(os.path.join(pardir, os.pardir))
RFID3DModelPathDict = {
    "RFD8500": os.path.join(pardir, 'data', 'antenna3dModel_RFD8500.txt'),
    "Zebra9600-AN720": os.path.join(pardir, 'data', 'antenna3dModel.txt')
}

# Localization flag settings
LOC_FLAG_NO_RAW_DATA = 0
LOC_FLAG_ONLY_INV = 1
LOC_FLAG_LOCALIZED = 2

antennaMaxRange = 3.4
DEBUGET_SWITCH = False
BEL_THES = 0.000000001


class rfh3DRBTagLoclizer(rfidbotRBLocalizeATag):
    '''
    * class: rfh3DRBTagLoclizer
    * based on the RFusion handheld raw data using recursive Bayesian updating to localize a tag in 3D
    '''
    def __init__(self, node: Node):
        #super().__init__()
        self.node = node
        self.name = "3D tag localizer"
        self.logger = get_logger('rfh3DRBTagLoclizer')

        # Defaults (these parameters were ROS1 params)
        self.minRfidObDis = 0.01
        self.readerType = "RFD8500"

        # Allow overriding the antenna model location at runtime
        try:
            self.declare_parameter('antenna_model_path', '')
        except Exception:
            # If running outside a Node context (e.g., unit tests), skip declaration
            pass

        if self.readerType not in RFID3DModelPathDict:
            self._logwarn("Reader type %s not valid. Defaulting to RFD8500", self.readerType)
            self.readerType = "RFD8500"

        self.RFID3DModel = None
        self.loadRFID3DModel()
        self.poseShifter = rfhposeShifter()
        self.debuger = RRRBDebuger()

    # --- small logging helpers ---
    def _logwarn(self, msg, *args):
        self.logger.warn(msg % args if args else msg)

    def _loginfo(self, msg, *args):
        self.logger.info(msg % args if args else msg)

    def loadRFID3DModel(self):
        # Ensure pickled antenna model can import its module without ROS1 rospy
        if 'rospy' not in sys.modules:
            sys.modules['rospy'] = types.SimpleNamespace()  # stub for ROS1 dependency
        try:
            # Make sure the pickle module name resolves
            if 'rfidbot_antenna_model_base' not in sys.modules:
                importlib.import_module('rfh_share_lib.rfidbot_antenna_model_base')
                sys.modules['rfidbot_antenna_model_base'] = sys.modules['rfh_share_lib.rfidbot_antenna_model_base']
        except Exception:
            # Best effort; fallback to whatever is on sys.path
            pass

        # Allow overriding via ROS param and try common install/share locations
        override_path = None
        try:
            override_path = self.get_parameter('antenna_model_path').get_parameter_value().string_value
        except Exception:
            pass

        candidate_paths = []
        if override_path:
            candidate_paths.append(override_path)
        candidate_paths.append(RFID3DModelPathDict[self.readerType])
        candidate_paths.append(os.path.join(root_dir, 'data', 'antenna3dModel_RFD8500.txt'))
        candidate_paths.append(os.path.join(root_dir, 'data', 'antenna3dModel.txt'))

        # Also check share directory if installed
        try:
            share_dir = get_package_share_directory('rfidbot_tags_localization')
            candidate_paths.append(os.path.join(share_dir, 'data', 'antenna3dModel_RFD8500.txt'))
            candidate_paths.append(os.path.join(share_dir, 'data', 'antenna3dModel.txt'))
        except Exception:
            pass

        loadAddr = None
        for p in candidate_paths:
            if p and os.path.isfile(p):
                loadAddr = p
                break

        if loadAddr is None:
            searched = "\n  - ".join(candidate_paths)
            raise FileNotFoundError(
                f"Antenna model file not found. Searched:\n  - {searched}\n"
                "Provide the model file (pickled antenna3dModel_*.txt) or set parameter 'antenna_model_path'."
            )

        with open(loadAddr, 'rb') as f:
            self.RFID3DModel = pickle.load(f)
            self._logwarn("Read antenna model from file %s", loadAddr)

        self._logwarn("Info: %s", self.RFID3DModel.AntennaInfo)
        self._logwarn("Resolution: %.2f", self.RFID3DModel.BeliefMapResolution)
        self._logwarn("mapWidth: %d", self.RFID3DModel.mapWidth)
        self._logwarn("mapHeight: %d", self.RFID3DModel.mapHeight)
        self._logwarn("mapVertical: %d", self.RFID3DModel.mapVertical)
        self._logwarn("AntLocInBelMap: (%d,%d,%d)",
                      *self.RFID3DModel.AntLocInBelMap)
        self._logwarn("AntennaLoc: (%.2f,%.2f,%.2f)",
                      *self.RFID3DModel.AntennaLoc)
        self._logwarn("Power level: %s", str(self.RFID3DModel.powerLevel))

    def init3DHypothesisLoc(self, rawData):
        initLocSet = []

        MinX = rawData[0].antennaPose.pose.pose.position.x
        MinY = rawData[0].antennaPose.pose.pose.position.y
        MinZ = rawData[0].antennaPose.pose.pose.position.z
        MaxX, MaxY, MaxZ = MinX, MinY, MinZ

        for rawItem in rawData:
            px = rawItem.antennaPose.pose.pose.position.x
            py = rawItem.antennaPose.pose.pose.position.y
            pz = rawItem.antennaPose.pose.pose.position.z
            MinX, MaxX = min(MinX, px), max(MaxX, px)
            MinY, MaxY = min(MinY, py), max(MaxY, py)
            MinZ, MaxZ = min(MinZ, pz), max(MaxZ, pz)

        offsetRange = antennaMaxRange * 0.5
        MinX -= offsetRange
        MinY -= offsetRange
        MaxX += offsetRange
        MaxY += offsetRange
        MinZ -= offsetRange
        MaxZ += offsetRange

        x = MinX
        while x < MaxX:
            x += self.RFID3DModel.BeliefMapResolution
            y = MinY
            while y < MaxY:
                y += self.RFID3DModel.BeliefMapResolution
                z = MinZ
                while z < MaxZ:
                    z += self.RFID3DModel.BeliefMapResolution
                    initLocSet.append([x, y, z])
        self._logwarn("init hypothesis location size %s", len(initLocSet))
        return initLocSet

    def filterOutRawData(self, rawData):
        newRawData = []
        for rawItem in rawData:
            if (rawItem.antennaPose and rawItem.antennaPose.pose and rawItem.antennaPose.pose.pose):
                newRawData.append(rawItem)
        return newRawData

    def generateTf(self, rawItem):
        """
        Return (trans, rot) tuple representing the antenna pose in the global frame.
        """
        antennaPose = rawItem.antennaPose
        if antennaPose is None or antennaPose.pose is None:
            return None
        trans = (
            antennaPose.pose.pose.position.x,
            antennaPose.pose.pose.position.y,
            antennaPose.pose.pose.position.z,
        )
        rot = (
            antennaPose.pose.pose.orientation.x,
            antennaPose.pose.pose.orientation.y,
            antennaPose.pose.pose.orientation.z,
            antennaPose.pose.pose.orientation.w,
        )
        return (trans, rot)

    def getBelin3DModel(self, rawItem, esLoc, trans, rot):
        """
        Compute belief value at esLoc given antenna pose (trans, rot).
        """
        zeroBel = 0.1
        # Rotate esLoc into antenna frame using quaternion, then translate by antenna pose.
        matrix = tf_transformations.quaternion_matrix(rot)
        pos = [esLoc[0], esLoc[1], esLoc[2], 0.0]
        postr = matrix.dot(pos)
        x = trans[0] + postr[0]
        y = trans[1] + postr[1]
        z = trans[2] + postr[2]

        model_x = y + self.RFID3DModel.AntennaLoc[0]
        model_y = x + self.RFID3DModel.AntennaLoc[1]
        model_z = z + self.RFID3DModel.AntennaLoc[2]

        mapIdCol = int(round(model_y / self.RFID3DModel.BeliefMapResolution))
        mapIdRow = int(round(model_x / self.RFID3DModel.BeliefMapResolution))
        mapIdVel = int(round(model_z / self.RFID3DModel.BeliefMapResolution))

        if not (0 <= mapIdCol < self.RFID3DModel.mapWidth and
                0 <= mapIdRow < self.RFID3DModel.mapHeight and
                0 <= mapIdVel < self.RFID3DModel.mapVertical):
            return zeroBel

        PLId = None
        for Id, PL in enumerate(self.RFID3DModel.powerLevel):
            if rawItem.powerLevel == PL:
                PLId = Id
                break
        if PLId is None:
            self._logwarn("cannot find powerlevel %s in %s",
                          rawItem.powerLevel, str(self.RFID3DModel.powerLevel))
            return zeroBel

        Bel = self.RFID3DModel.BinaryMap[PLId][mapIdRow][mapIdCol][mapIdVel]
        return Bel if Bel != 0 else zeroBel

    def projectARawItem2Map(self, rawItem, trans, rot):
        for id, esLoc in enumerate(self.estimatedLocation):
            meas = self.getBelin3DModel(rawItem, esLoc, trans, rot)
            self.antennaBelInmap[id]['bel'] = meas
        self._logwarn("project (%.3f,%.3f,%.3f) model to map",
                      rawItem.antennaPose.pose.pose.position.x,
                      rawItem.antennaPose.pose.pose.position.y,
                      rawItem.antennaPose.pose.pose.position.z)
        self.debuger.visualBelin3DMap(self.antennaBelInmap, 'y')
        self.debuger.visualARawDatain3Dmap(rawItem)

    def belUpdate(self, id, esLoc, rawItem, trans, rot):
        if self.esLocBel[id]['bel'] < BEL_THES:
            return
        meas = self.getBelin3DModel(rawItem, esLoc, trans, rot)
        self.esLocBel[id]['bel'] *= meas

    def normalizeBelByMaxValue(self):
        maxBel = max([b['bel'] for b in self.esLocBel], default=0)
        if maxBel != 0:
            for b in self.esLocBel:
                b['bel'] /= maxBel

    def estimatePosByAverageThresBel(self, esLocBel, Thres):
        belSum = 0
        posx = posy = posz = 0
        for esLoc in esLocBel:
            if esLoc['bel'] > Thres:
                posx += esLoc['bel'] * esLoc['esloc'][0]
                posy += esLoc['bel'] * esLoc['esloc'][1]
                posz += esLoc['bel'] * esLoc['esloc'][2]
                belSum += esLoc['bel']
        if belSum != 0:
            return (posx / belSum, posy / belSum, posz / belSum)
        return (None, None, None)

    def initTf2ForAntennaPose(self):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        return (tf_buffer, tf_listener)

    def tfCameraPose2AntennaPose(self, tf_buffer, rData):
        targetFrame = "RFD8500_antena"
        sourceframe = "ZED_center"
        globalFrame = "map"
        rData.antennaPose.pose = self.poseShifter.shiftPose(
            sourceframe, targetFrame, globalFrame, rData.antennaPose
        )

    def get2RawDataAntennaDis2(self, rawdataItem1, rawdataItem2):
        p1 = rawdataItem1.antennaPose.pose.pose.position
        p2 = rawdataItem2.antennaPose.pose.pose.position
        return (p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2

    def deduplicateRawdata(self, rawData, dedupDis):
        if len(rawData) < 2:
            return rawData
        dedupDis2 = dedupDis**2
        deduRawdata = [rawData[0]]
        for rawdataItem in rawData[1:]:
            minDis2 = min(self.get2RawDataAntennaDis2(d, rawdataItem) for d in deduRawdata)
            if minDis2 > dedupDis2:
                deduRawdata.append(rawdataItem)
        return deduRawdata

    def localizesATag(self, tagEPC, rawData):
        self._logwarn("localizing tag %s", tagEPC)
        if rawData is None:
            self._logwarn("the rawData is None!")
            return (LOC_FLAG_NO_RAW_DATA, None, None, None)

        self._logwarn("the raw data size %s", len(rawData))
        (tf_buffer, tf_listener) = self.initTf2ForAntennaPose()

        filteredRawData = self.filterOutRawData(rawData)
        filteredRawData = self.deduplicateRawdata(filteredRawData, self.minRfidObDis)
        self._logwarn("the valid raw data size %s (minDis %s)",
                      len(filteredRawData), self.minRfidObDis)
        if len(filteredRawData) < 1:
            return (LOC_FLAG_ONLY_INV, None, None, None)

        self.estimatedLocation = self.init3DHypothesisLoc(filteredRawData)
        self.initEsLocBel()

        i = 0
        for rData in filteredRawData:
            if DEBUGET_SWITCH:
                self.debuger.visualARawDatain3Dmap(rData)
                time.sleep(5)

            g2m_tf = self.generateTf(rData)
            if g2m_tf is None:
                continue
            trans, rot = g2m_tf

            i += 1
            for idx, esLoc in enumerate(self.estimatedLocation):
                self.belUpdate(idx, esLoc, rData, trans, rot)

            if DEBUGET_SWITCH:
                self.debuger.visualBelin3DMap(self.esLocBel)
                self.debuger.visualARawDatain3Dmap(rData)
                time.sleep(3)

        self.normalizeBelByMaxValue()
        if DEBUGET_SWITCH:
            self.debuger.visualBelin3DMap(self.esLocBel)

        Thres = 0.2
        (posx, posy, posz) = self.estimatePosByAverageThresBel(self.esLocBel, Thres)
        self._logwarn("localize %s is done!", tagEPC)
        return (LOC_FLAG_LOCALIZED, posx, posy, posz)
