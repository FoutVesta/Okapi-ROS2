#!/usr/bin/python3
'''
*File name: rfidbot_tags_localization_rawdata_recorder_base.py
*Description: provides the basic data structure of tag localization rawdata recorder
*Author: Jian zhang
*Create date: Feb/15/2016
'''

import rospy
from std_msgs.msg import Int16

from rfidbot_tags_reader.msg import tagReader
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

import pickle
import os

FLOAT_ZERO = 0.00000001
BAD_POSITION = 9999.9

'''
class Name: rfidbotTagLocRawDataRecordBase
'''
class rfidbotTagLocRawDataRecordBase():
    def __init__(self):
        self.tags = []
        self.uniqueTagsEPC = []  #the unique tags, must be called the getUuiqueTags function to generate
        self.postions = []
        self.currentPose = None
        self.tagLocRawData = None
        self.rawDataFileAddr = None

        rospy.Subscriber("/rfid_tags", tagReader, self.rfidtagsCallBack) 
        #rospy.Subscriber("pose", PoseWithCovarianceStamped, self.poseCallback)
        rospy.Subscriber("/odom", Odometry, self.poseCallback)
        
    #Just samply add the tag into a arry        
    def rfidtagsCallBack (self,msg):
        msg.EPC = msg.EPC.lower()
        self.tags.append(msg)
        rospy.logwarn("tag  %s",len(self.tags))

    def getUuiqueTags(self):
        '''
        * Description: from all record rfid tags to generate a unique tags list
        * Input: None
        * Output: None
        '''
        rospy.logwarn("tag number: %s",len(self.tags))
        for tag in self.tags:
            rospy.logwarn("tag %s",tag.EPC )
            if tag.EPC not in self.uniqueTagsEPC:
                self.uniqueTagsEPC.append(tag.EPC )
        rospy.logwarn("Total scanned unique tag number: %s",len(self.uniqueTagsEPC))
        
    def isPoseValid(self,pose):
        '''if (abs(pose.pose.pose.position.x) < FLOAT_ZERO and
           abs(pose.pose.pose.position.y) < FLOAT_ZERO and
           abs(pose.pose.pose.position.z) < FLOAT_ZERO and
           abs(pose.pose.pose.orientation.x) < FLOAT_ZERO and
           abs(pose.pose.pose.orientation.y) < FLOAT_ZERO and
           abs(pose.pose.pose.orientation.z) < FLOAT_ZERO and
           abs(pose.pose.pose.orientation.w) < FLOAT_ZERO ):'''
        if(pose.pose.pose.position.x == BAD_POSITION and 
        pose.pose.pose.position.y == BAD_POSITION and 
        pose.pose.pose.position.z == BAD_POSITION and 
        pose.pose.pose.orientation.x == BAD_POSITION and 
        pose.pose.pose.orientation.y == BAD_POSITION and 
        pose.pose.pose.orientation.z == BAD_POSITION and 
        pose.pose.pose.orientation.w == BAD_POSITION ):
            #alarm for lost post
            #os.system('play --no-show-progress --null --channels 1 synth %s sine %f' % (0.1, 800))
            return False
        else:
            return True
            
    def poseCallback(self,msg):
        if not self.isPoseValid(msg):
            self.currentPose = None
            #rospy.logwarn("get pose is Null")
            return
        self.currentPose = msg
        #rospy.logwarn("get pose %s",self.currentPose.pose.pose.position.x)
        self.postions.append(msg)

    def saveRawData2File(self):
        if self.rawDataFileAddr == None:
            rospy.logwarn('the file address is None!')
            return
            
        with open(self.rawDataFileAddr, 'wb') as f:
            pickle.dump(self.tagLocRawData, f)
        rospy.logwarn("recorder:save raw data to file %s",self.rawDataFileAddr)

    def readRawDataFromFile(self):
        if self.rawDataFileAddr == None:
            rospy.logwarn('the file address is None!')
            return
            
        with open(self.rawDataFileAddr, 'rb') as f:
            self.tagLocRawData = pickle.load(f)
            #self.tagLocRawData = self.tagLocRawData[0:6186]
        if self.tagLocRawData != None:
            recordLength = len(self.tagLocRawData)
        else:
            recordLength = 0
        rospy.logwarn("recorder:read raw data from file %s, with length %s",
                    self.rawDataFileAddr,recordLength)
                
        #self.tagLocRawData = self.correctReatRatePose(self.tagLocRawData)
                                
    def setRawDataFileAdr(self,fileAddr):
        self.rawDataFileAddr = fileAddr


    def getRawDataForATag(self,TagEpc):
        '''
        * description: return the raw data for a tag 
        * input: TagEpc, the EPC of tag
        * output: return the rawdata
        '''

        RawData = []
        if (self.tagLocRawData == None):
           return None 

        for id,rawData in enumerate(self.tagLocRawData):
            if TagEpc == rawData.TagEpc:
                RawData.append(rawData)
        if len(RawData) < 1:
            return None

        return RawData

