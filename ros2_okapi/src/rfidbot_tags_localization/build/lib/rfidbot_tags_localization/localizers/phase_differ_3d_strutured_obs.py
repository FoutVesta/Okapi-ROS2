#!/usr/bin/env python3
'''
* File name: phase_differ_3d_strutured_obs.py
* Description: This file provides a tree like data structure of all obdervations for a tag,
                observations
              ________|________
             |                 |   
             EPC              rawObsData  (for all antenna)                                             ------- 
                        ________|________   ...      ________|________                                  level1(obsAnt)
                       |                 |          |                 |
                 Antenna ID       obs (all observatons for this antenna)                                ------- 
                                 ________|________  ...      ________|________                          level2(obsPose)
                                |                 |         |                 |
                        antenna pose        obs (all observatons for this pose)                         ------ 
                                             ________|________   ...                                    level3(obsChannel)
                                            |                 |         
                                        Channel ID         obs (all observatons for this channel)       -------
                                                              |        ...  
                                                        Oberserations  (objects of rfidbotTagsLocReadRateRawData )
* Author: Jian Zhang
* Create Date: 2018-3-5
* Modified Date: N0v/10/2025 by Justin Palm
* Version 2.0 for ROS2 Humble
* Note: logic preserved exactly — including the known antenna indexing behavior
'''

import rclpy
from rclpy.logging import get_logger
import copy

FLOAT_ZERO = 0.00000001

class structedObservations():
    def __init__(self):
        self.EPC = None
        self.rawObsData = None
        self.structedObsData = []          # hierarchical data tree
        self.normalizedStrObsData = []     # normalized version
        self.logger = get_logger('structedObservations')

    def setRawData(self, EPC, rawData):
        '''
        * Description: set the raw observation data for a tag
        * Input: EPC, rawData
        '''
        self.EPC = EPC
        self.rawObsData = rawData
        self.bulidStructedObs()

    def getLev1ObsbyAntennaId(self, obsLevel1List, antennaID):
        '''
        * Description: get the level1 obs for a specific antennaID
        '''
        for obsLevel1 in obsLevel1List:
            if str(obsLevel1["antennaID"]) == str(antennaID):
                return obsLevel1
        return None

    def getLev2ObsbyPose(self, obsLevel1, antennaPose):
        '''
        * Description: get the level2 obs for a specific pose
        '''
        for obsLevel2 in obsLevel1["obs"]:
            if self.isSamePose(obsLevel2["antennaPose"], antennaPose):
                return obsLevel2
        return None

    def getLev3ObsbyChannelID(self, obsLevel2, channelID):
        '''
        * Description: get the level3 obs for a specific channelID
        '''
        for obsLevel3 in obsLevel2["obs"]:
            if str(obsLevel3["channelID"]) == str(channelID):
                return obsLevel3
        return None

    def newLevel3Obs(self):
        '''
        * Description: create a level3 observation list with 50 empty entries
        '''
        obsLevel3 = []
        for i in range(0, 50):
            obsLevel3.append({"channelID": str(i), "obs": []})
        return obsLevel3

    def isSamePose(self, pose1, pose2):
        '''
        * Description: check if two poses are same within tolerance
        '''
        if abs(pose1.pose.pose.position.x - pose2.pose.pose.position.x) < 0.005 and \
           abs(pose1.pose.pose.position.y - pose2.pose.pose.position.y) < 0.005 and \
           abs(pose1.pose.pose.position.z - pose2.pose.pose.position.z) < 0.005:
            return True
        else:
            return False

    def bulidStructedObs(self):
        '''
        * Description: build the structured observation tree
        * Level 1: antennaID
        * Level 2: antennaPose
        * Level 3: channelID
        * Level 4: raw observation data
        '''
        self.structedObsData = []
        for rawObs in self.rawObsData:
            antennaID = rawObs.antennaID
            antennaPose = rawObs.antennaPose
            channelID = rawObs.channelID

            obsLevel1 = self.getLev1ObsbyAntennaId(self.structedObsData, antennaID)
            if obsLevel1 == None:
                self.structedObsData.append({"antennaID": str(antennaID), "obs": []})
                obsLevel1 = self.structedObsData[0]   # <— original behavior preserved

            obsLevel2 = self.getLev2ObsbyPose(obsLevel1, antennaPose)
            if obsLevel2 == None:
                obsLevel2 = {"antennaPose": antennaPose, "obs": self.newLevel3Obs()}
                obsLevel1["obs"].append(obsLevel2)

            obsLevel3 = self.getLev3ObsbyChannelID(obsLevel2, channelID)
            if obsLevel3 == None:
                obsLevel3 = {"channelID": str(channelID), "obs": []}
                obsLevel2["obs"].append(obsLevel3)

            obsLevel3["obs"].append(rawObs)

        self.logger.info("Structured observations built for EPC %s" % self.EPC)

    def normalizedObservations(self, channelSampleNum=10):
        '''
        * Description: normalize the number of observations per channel
        '''
        self.normalizedStrObsData = copy.deepcopy(self.structedObsData)
        for obsLevel1 in self.normalizedStrObsData:
            for obsLevel2 in obsLevel1["obs"]:
                for obsLevel3 in obsLevel2["obs"]:
                    obsList = obsLevel3["obs"]
                    if len(obsList) > channelSampleNum:
                        obsLevel3["obs"] = obsList[:channelSampleNum]
        self.logger.info("Normalized observations for EPC %s" % self.EPC)

    def printStatisticInfo(self):
        '''
        * Description: print out statistics of structured observations
        '''
        antennaNum = len(self.structedObsData)
        poseNum = 0
        obsNum = 0
        for obsLevel1 in self.structedObsData:
            poseNum += len(obsLevel1["obs"])
            for obsLevel2 in obsLevel1["obs"]:
                for obsLevel3 in obsLevel2["obs"]:
                    obsNum += len(obsLevel3["obs"])

        self.logger.info("Structured Observation Stats:")
        self.logger.info("  Total antennas: %s" % antennaNum)
        self.logger.info("  Total poses: %s" % poseNum)
        self.logger.info("  Total observations: %s" % obsNum)

    def getNorObs(self):
        '''
        * Description: get normalized observation data
        '''
        return self.normalizedStrObsData

