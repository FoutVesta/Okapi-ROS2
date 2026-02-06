#!/usr/bin/env python3
'''
 * File Name: recursive_bayes_localizer_gpu_3d.py
 *
 * Description: based on the Okapi raw tag reading location in 3d, use a probabilistic
 *              approach based on recursive Bayesian updating to localize a tag in 3d.
 *              It will use GPU to optimize the process.
 *
 * Author: Jian Zhang
 * Create Date: 2020-8-9
 * Modified Date: Nov/10/2025 by Justin Palm
 * Version: 2.0 for ROS2 Humble
 * Remark:
'''

import sys, os
import pickle
import time
from timeit import default_timer as timer
import numpy as np

import rclpy
from rclpy.logging import get_logger

from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped
import tf_transformations  # quaternion math helpers

import tf2_ros
import tf2_geometry_msgs

# ---- Absolute imports inside your ROS2 package ----
from rfidbot_tags_localization.localizers.rfidbot_tags_recursive_bayes_localize import rfidbotRBLocalizeATag
from rfidbot_tags_localization.localizers.rfh_3d_recursive_bayes_localizer import rfh3DRBTagLoclizer
from rfidbot_tags_localization.libs.rfh_tags_localization_pose_shifter import rfhposeShifter
from rfidbot_tags_localization.libs.rfidbot_tags_debuger import RRRBDebuger

path = sys.path[0]
pardir = os.path.abspath(os.path.join(path, os.pardir))
# RFD8500 reader model
RFID3DModelPath = pardir + '/data/antenna3dModel_RFD8500.txt'

# Localization flags
LOC_FLAG_NO_RAW_DATA = 0    # no raw data
LOC_FLAG_ONLY_INV    = 1    # only inventory, cannot localize
LOC_FLAG_LOCALIZED   = 2    # localized

antennaMaxRange = 3.4
DEBUGET_SWITCH = False
BEL_THES = 0.000000001


class GPU3DRBTagLoclizer(rfh3DRBTagLoclizer):
    '''
    * class: GPU3DRBTagLoclizer
    * based on the Okapi raw data using recursive Bayesian updating to localize a tag in 3d
    * and optimized by GPU
    '''
    def __init__(self):
        self.name = "GPU 3d tag localizer"
        super().__init__()
        self.logger = get_logger('GPU3DRBTagLoclizer')
        # self.minRfidObDis is defined in base; ensure it exists / default if needed
        if not hasattr(self, 'minRfidObDis'):
            self.minRfidObDis = 0.01

    # --- small helpers to mirror rospy logging ---
    def _logwarn(self, msg, *args):
        self.logger.warn(msg % args if args else msg)

    def _loginfo(self, msg, *args):
        self.logger.info(msg % args if args else msg)

    def generateTf(self, rawItem):
        '''
        * description: given raw item and generate tf
        *              The tf transforms a model at origin/zero-orientation to the pose in the global frame
        * output: return (trans, rot) or (None, None) if failed
        '''
        antennaPose = rawItem.antennaPose
        if antennaPose is None or antennaPose.pose is None:
            return None, None

        # We only need a static transform (translation + rotation) from global -> antenna.
        # Use the pose directly instead of the old ROS1 TransformerROS helper.
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
        '''
        * description: given raw item and location in global frame, find out the belief of this point
        * input: rawItem, the raw data item
                 esLoc, the global location, esLoc[0],esLoc[1],esLoc[2] -> x,y,z
        * output: return bel of this point
        '''
        zeroBel = 0.1

        # Use quaternion to rotate esLoc into model frame then translate by trans
        matrix = tf_transformations.quaternion_matrix(rot)
        pos = np.array([esLoc[0], esLoc[1], esLoc[2], 0])  # homogeneous with w=0 for direction-like transform
        rotMatrix = np.array(matrix)
        postr = np.dot(rotMatrix, pos.T)
        x = trans[0] + postr[0]
        y = trans[1] + postr[1]
        z = trans[2] + postr[2]

        # convert reader antenna center frame to rfid model frame
        model_x = y + self.RFID3DModel.AntennaLoc[0]
        model_y = x + self.RFID3DModel.AntennaLoc[1]
        model_z = z + self.RFID3DModel.AntennaLoc[2]
        # convert the rfid model frame location into row,col,vertical
        mapIdCol = int(round(model_y / self.RFID3DModel.BeliefMapResolution))
        mapIdRow = int(round(model_x / self.RFID3DModel.BeliefMapResolution))  # image row is x axis
        mapIdVel = int(round(model_z / self.RFID3DModel.BeliefMapResolution))

        if mapIdCol < 0 or mapIdCol > self.RFID3DModel.mapWidth - 1:
            return zeroBel
        if mapIdRow < 0 or mapIdRow > self.RFID3DModel.mapHeight - 1:
            return zeroBel
        if mapIdVel < 0 or mapIdVel > self.RFID3DModel.mapVertical - 1:
            return zeroBel

        PLId = None
        for Id, PL in enumerate(self.RFID3DModel.powerLevel):
            if rawItem.powerLevel == PL:
                PLId = Id
                break
        if PLId is None:
            self._logwarn("can not find right powerlevel %s in modeled power level: %s",
                          str(rawItem.powerLevel), str(self.RFID3DModel.powerLevel))
            return zeroBel

        Bel = self.RFID3DModel.BinaryMap[PLId][mapIdRow][mapIdCol][mapIdVel]
        if Bel == 0:
            Bel = zeroBel
        return Bel

    # Jian add for test
    # @autojit
    def belUpdate(self, id, esLoc, rawItem, trans, rot):
        '''
        * Description: override the belUpdate to support 3D
        * input:
        *       id, the id of estimated position in the esLocBel list
                esLoc, the estimated position in the map (x,y,z)
                rawItem, raw data Item
        '''
        if self.esLocBel[id]['bel'] < BEL_THES:
            return
        meas = self.getBelin3DModel(rawItem, esLoc, trans, rot)
        newBel = self.esLocBel[id]['bel'] * meas
        self.esLocBel[id]['bel'] = newBel

    # Jian add for test
    # @autojit
    def localizesATag(self, tagEPC, rawData):
        '''
        * Description: localize a tag by given EPC and related rawdata
        * input: tagEPC, the EPC of tag
                 rawData, the rawdata of a tag
        * output: return, localization flag: LOC_FLAG_NO_RAW_DATA / LOC_FLAG_ONLY_INV / LOC_FLAG_LOCALIZED
                          posx, posy, posz  tag location in map
        '''
        self._logwarn("localizing tag %s", tagEPC)

        if rawData is None:
            self._logwarn("the rawData is None!")
            return (LOC_FLAG_NO_RAW_DATA, None, None, None)

        self._logwarn("the raw data with size %s", len(rawData))

        # filter the rawdata
        filteredRawData = self.filterOutRawData(rawData)
        filteredRawData = self.deduplicateRawdata(filteredRawData, self.minRfidObDis)
        self._logwarn("the valid raw data size %s, minRfidObDis %s", len(filteredRawData), self.minRfidObDis)
        if len(filteredRawData) < 1:
            return (LOC_FLAG_ONLY_INV, None, None, None)

        # initial hypothesis locations and belief
        self.estimatedLocation = self.init3DHypothesisLoc(filteredRawData)
        self.initEsLocBel()

        # recursively updating
        i = 0
        for rData in filteredRawData:
            if DEBUGET_SWITCH:
                self._logwarn("before  %d of %d, rawdata[%s](%.3f, %.3f, %.3f)",
                              i, len(filteredRawData), rData.antennaID,
                              rData.antennaPose.pose.pose.position.x,
                              rData.antennaPose.pose.pose.position.y,
                              rData.antennaPose.pose.pose.position.z)
                self.debuger.visualARawDatain3Dmap(rData)
                time.sleep(10)

            (trans, rot) = self.generateTf(rData)  # transform of RFID model with given pose in global frame
            if trans is None or rot is None:
                continue

            if DEBUGET_SWITCH:
                self._logwarn("recursively updating %d of %d, rawdata(%.3f, %.3f, %.3f)",
                              i, len(filteredRawData),
                              rData.antennaPose.pose.pose.position.x,
                              rData.antennaPose.pose.pose.position.y,
                              rData.antennaPose.pose.pose.position.z)

            i += 1

            for idx, esLoc in enumerate(self.estimatedLocation):
                self.belUpdate(idx, esLoc, rData, trans, rot)

            if DEBUGET_SWITCH:
                self.debuger.visualBelin3DMap(self.esLocBel)
                self.debuger.visualARawDatain3Dmap(rData)
                time.sleep(5)

        # normalize all the bel at the end
        self.normalizeBelByMaxValue()
        if DEBUGET_SWITCH:
            self.debuger.visualBelin3DMap(self.esLocBel)

        Thres = 0.2
        (posx, posy, posz) = self.estimatePosByAverageThresBel(self.esLocBel, Thres)
        self._logwarn("localize %s is done!", tagEPC)

        return (LOC_FLAG_LOCALIZED, posx, posy, posz)
