#!/usr/bin/python3
'''
*File name: rfidbot_tags_localization_readrate_rawdata_base.py
*Description: provide the data class for raw data of read rate recording
*Author: Jian zhang
*Create date: Oct/30/2015
'''

import rospy

'''
*class Name rfidbotTagsLocReadRateRawData
# this class is design for robot stop at a position to reading the read rate
# very tag with same antenna ID in a reading period will occupy a object of this class
'''
    
class rfidbotTagsLocReadRateRawData():
    def __init__(self):
        self.TagEpc = None 
        self.antennaID = None
        self.antennaPose = None  # the antenna position 
        self.readRate = None
        self.readAmount = None
        self.firstReadTimeStamp = None
        self.lastReadTimeStamp = None
        self.readingPeriod = None
        self.environmentSacnnedTagsNum = None
        self.powerLevel = None # powerLevel = 80...
        self.phase = None # this is string
        self.channelID = None # string
        self.peakRSSI = None #string

    def updateReadRate(self):
        if self.readingPeriod != 0:
            self.readRate = self.readAmount / self.readingPeriod

class rfidbotTagsLocReadRateVectorRawData():
    '''
     * description: this class provide the all power level read rate which in same position adn 
                    same antenna will be recorded in a vector
    '''
    def __init__(self):
        self.TagEpc = None 
        self.antennaID = None
        self.antennaPose = None  # the antenna position 
        self.powerLevel = None # powerLevel = [80,...]
        self.readRateVector = None #[rr80,rr110,...]
