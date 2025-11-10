#!/usr/bin/python3
'''
*File name: rfh_tags_localization_fix_power_rawdata_recorder.py
*Description: record the tag localization raw data and combine with reader pose
              the reader is working under fix power
*Author: Jian zhang
*Create date: Aug/10/2016
'''

import rospy
from std_msgs.msg import Bool
import sys,os 
import os.path

# sys.path.append(os.path.join(os.path.dirname(__file__), '../../rfid_tags_reader'))
from rfidbot_tags_reader.msg import tagReader

sys.path.append(os.path.join(os.path.dirname(__file__), '../data_recorders'))
from rfidbot_tags_localization_readrate_rawdata_base import rfidbotTagsLocReadRateRawData
from rfidbot_tags_localization_rawdata_recorder_base import rfidbotTagLocRawDataRecordBase

sys.path.append(os.path.join(os.path.dirname(__file__), '../../../'))
from rfh_share_lib.rfidbot_set_para_2_reader import rfidbotReaderFilterSetter
from libs.rfh_tags_localization_pose_shifter import rfhposeShifter
from nav_msgs.msg import Odometry


#for debuger
#from rfidbot_tags_debuger import RRRBDebuger

import pickle

#jian add to set the power to zebra 9600
#from zebra_rfid_reader.msg import control_input, write_data



class rfhFixPowerRecoder(rfidbotTagLocRawDataRecordBase):  
    def __init__(self,rateRatio,rate):
        rfidbotTagLocRawDataRecordBase.__init__(self)
        self.name = "rfh fix power raw data recorder"

        self.rateRatio = rateRatio    #the node spin rate ratioe
        self.rate = rate  #the rate
        
        self.tagLocRawData = []

        #set the parameter from the launch file, otherwise use the default one
        self.power = int(rospy.get_param('~txpower',130))  # 130 is the default one
        self.antennaFrameId = rospy.get_param('~antenna_frameid',"RFD8500_antena")   #This is the parameter for RFD 8500 only
        self.mapFrameId = rospy.get_param('~map_frameid',"map")

        self.poseShifter = rfhposeShifter()
        
        #init filtersetter for power reset and tag filter reset
        #TO do: reset power here the only purpose is get the power level of the reader 
        #       we can add the power level to the reader reports, thus we can reduce the reset power here.
        # = 180 #for RFD8500 is 180
        self.isResetPower = False
        self.filterSetter = rfidbotReaderFilterSetter(self.rateRatio,self.rate)

        rospy.Subscriber("/set_tx",Bool,self.setPowerCallback)
        self.waitForPeriod(2.5)
        self.resetPowerLevel2Reader()
        
#        self.debuger = RRRBDebuger()

    def setPowerCallback(self,msg):
        if (msg.data):
            self.resetPowerLevel2Reader()
            
    def resetPowerLevel2Reader(self):
        rospy.logwarn("set power level %s ",self.power)
        self.filterSetter.pauseInven2DelRos()
        self.waitForPeriod(1.5)
        self.filterSetter.setTxPower(self.power)
        self.waitForPeriod(1.5)
        self.filterSetter.resumeInvenWithNewRos()
        
        self.isResetPower = True

        #jian add to set the power to zebra 9600
        
        
    def  waitForPeriod(self, idleSeconds):
        '''
        *function Name: waitForPeriod
        *description: system wait for a period
        *input: 
                idleSeconds: the wait period in seconds
        *return: None
        '''  
        if idleSeconds == None:
            return 
        for i in range(0,int(self.rateRatio * idleSeconds)):
            self.rate.sleep()    
            
    def rfidtagsCallBack (self,msg):
        '''
        *   Description: rewrite the based rfidtagsCallBack to generate a rfid record with pose
        '''
        if(not self.isResetPower):
            return
        msg.EPC = msg.EPC.lower()
        tag = msg
        candidatePose = self.currentPose
        tmpRawData = self.initARawDataByTagandPose(tag,candidatePose)
        self.tagLocRawData.append(tmpRawData)


    def getUuiqueTags(self):
        '''
        * Description: from all record rfid tags to generate a unique tags list
        * Input: None
        * Output: None
        '''
        for id,rawData in enumerate(self.tagLocRawData):
            if rawData.TagEpc not in self.uniqueTagsEPC:
                self.uniqueTagsEPC.append(rawData.TagEpc )
        rospy.logwarn("Total scanned unique tag number: %s",len(self.uniqueTagsEPC))

    def saveRawData2File(self, rawDataFile = None):
        '''
        * description: save the rawdata into file
        * input: rawDataFile: if none save to default file, if the file is existing it will be rewrite
                              if not none save the rawdata into this file, it should include absolute address
        '''
        if rawDataFile == None:
            rawDataFile = self.rawDataFileAddr
        
        with open(rawDataFile, 'wb') as f:
            pickle.dump(self.tagLocRawData, f)
        rospy.logwarn("save raw data to file %s",rawDataFile)

        
        
    def readRawDataFromFile(self,rawDataFile = None):
        '''
        * description, read the rawdata from the file
        * rawDataFile: if none read the default file, 
                        if not none read the rawdata from this file, it should include absolute address
        '''
        if rawDataFile == None:
            rawDataFile = self.rawDataFileAddr
        if not os.path.isfile(rawDataFile):
            rospy.logwarn("raw data file %s is not existing",rawDataFile)
            return 
            
        with open(rawDataFile, 'rb') as f:
            self.tagLocRawData = pickle.load(f)
            rospy.logwarn("read raw data from file %s",rawDataFile)
            rospy.logwarn("the size of raw data %s",len(self.tagLocRawData))
        
            
            
    def initARawDataByTagandPose(self,tag,candidatePose):
        '''
        * description: by the item record of "rfid_tags" to initial a read rate raw data
        * input:   tag, a record of "rfid_tags", tagReader
                   candidatePose: the pose of robot
        * output:  rawData, a record of read rate raw data, which the read rate is set to 1
        '''
        #for debug
#        tmpRawData1 = rfidbotTagsLocReadRateRawData()  
#        tmpRawData1.antennaPose = candidatePose
#        self.debuger.visualARawDatain3Dmap_1(tmpRawData1,'g')
        
        tmpRawData = rfidbotTagsLocReadRateRawData()     
        tmpRawData.TagEpc = tag.EPC
        tmpRawData.antennaID = tag.AntennaID
        #rospy.logwarn("tag.AntennaID %s",tag.AntennaID) # RFD8500_1 for RFD 8500
        tmpRawData.antennaPose = self.tfCameraPose2AntennaPose(candidatePose,tmpRawData.antennaID)
        tmpRawData.readRate = 1
        tmpRawData.powerLevel = self.power
        tmpRawData.phase = tag.Phase
        tmpRawData.channelID = tag.Channel  #update the new msg file
        tmpRawData.peakRSSI = tag.PeakRSSI

#        #for debug
#        self.debuger.visualARawDatain3Dmap(tmpRawData)
        '''
        if candidatePose != None:
            if int(tmpRawData.channelID) == 28 and tmpRawData.TagEpc== '3034031dfc5f936d07a64190':  #for phase localiztion test
                rospy.logwarn("tag: %s,pose(%.3f,%.3f,%.3f),phase %s,ch:%s,rssi:%s",tmpRawData.TagEpc,
                          tmpRawData.antennaPose.pose.pose.position.x,
                          tmpRawData.antennaPose.pose.pose.position.y,
                          tmpRawData.antennaPose.pose.pose.position.z,
                          tmpRawData.phase,tmpRawData.channelID,tmpRawData.peakRSSI)
        else:
            if int(tmpRawData.channelID) == 28 and tmpRawData.TagEpc== '3034031dfc5f936d07a64190':  #for phase localiztion test
                rospy.logwarn("tag %s,power:%s,pose is None,phase %s,ch:%s,rssi:%s",
                            tmpRawData.TagEpc,tmpRawData.powerLevel,
                            tmpRawData.phase,tmpRawData.channelID,tmpRawData.peakRSSI)
        '''
        
        return tmpRawData


    def tfCameraPose2AntennaPose(self,candidatePose, antennaID):
        '''
        * Description: change the camera pose to antenna pose
        * input: candidatePose, the camera/okapi pose
                 antennaID, the antenna Id, for the RFD8500 is fixed as RFD8500_1
        * output: antenna pose
        '''
        if candidatePose == None:
            return None
        if "RFD8500_1" == antennaID:
            targetFrame = self.antennaFrameId # "RFD8500_antena"
        else:  #for fix reader the RFID_antena_x , x is the antenna ID
            targetFrame = "RFID_antena_" + antennaID 
        #rospy.logwarn("targetFrame %s",targetFrame) # RFD8500_1 for RFD 8500
        sourceframe = candidatePose.child_frame_id #"ZED_center" 
        globalFrame = self.mapFrameId #"map"
        #def shiftPose(self,sourceFrameId,targetFrameId,globalFrameId,pose):
        newpose = Odometry()
        newpose.pose =self.poseShifter.shiftPose(sourceframe,
                                         targetFrame,
                                         globalFrame,
                                         candidatePose)
        return newpose