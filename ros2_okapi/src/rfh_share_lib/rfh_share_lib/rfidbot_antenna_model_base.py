#!/usr/bin/env python
'''
*File name: rfidbot_antenna_model_base.py
*Description: provide base class for others
*Author: Jian zhang
*Create date: Oct/29/2015
*================================================================================================
*import instruction: 
   1. add an empty __init__.py in the destination folder
   2. add follows code in the destination file and then import
   import sys,os 
   sys.path.append('/home/rfid/catkin_ws/src/rfidbot_share_lib')  #this must be absolute address
*================================================================================================
'''

import rospy
import numpy as np
import cv2
from matplotlib import pyplot as plt
import math

class rfidbotRFIDModelBase:
    def __init__(self):
        self.AntennaInfo = None
        self.BeliefMapResolution = None 
        self.BeliefMap = None         #[[powerlevel1][rr1][x][y],.... if 3d [[powerlevel1][x][y][z],
        self.mapHeight = None         #in y axis
        self.mapWidth  = None         #in x axis
        self.mapVertical = None       #in z axis, for 3D only
        self.AntennaLoc  = None       #the antenna center location in the map
        self.AntLocInBelMap  = None   #the antenna center location belief map is the clo and row in matrix
        self.antennaLevelBel = None   #this is not used in experiment based [[0,20],[1,15],[1.5,10],[2,5]]  #[[center, centerbel],[level 1, bel1],[level 2, bel2]...]
        self.zeroBel = 0.001
        
        #add for multiple power level
        self.powerLevel = None # [power1,power2,power3,...]
        self.BinaryMap  = None # represent the belief by binary, if in a position has any reading is 1, othervise is 0
                               # [[powerlevel1][x][y],... if 3d [[powerlevel1][x][y][z],

    def rotateReadRateModel(self,rotDegree,readRate,powerLevel):
        '''
        * description: rotate read rate model by sepsific degree
        * input: rotDegree, rotate degree (like 180 or 90 or 360)
                 readRate, read rate of model, the caller should guarantee it is correct
                 powerLevel, power level of the model,the caller should guarantee it is correct
        * return: the rotated model         
        '''
        rows = self.mapHeight
        cols = self.mapWidth 
        rotateCenterX = self.AntLocInBelMap[0]
        rotateCenterY = self.AntLocInBelMap[1]
        #find the right power level id
        for Id,PL in enumerate(self.powerLevel):
            if powerLevel == PL:
                PLId = Id
                break                    
        M = cv2.getRotationMatrix2D((rotateCenterX,rotateCenterY),rotDegree,1)
        dst = cv2.warpAffine(self.BeliefMap[PLId][readRate],M,(cols,rows))
        #plt.subplot(211),plt.imshow(self.BeliefMap[PLId][readRate])
        #plt.subplot(212), plt.imshow(dst)#,cmap = 'gray')
        #plt.show()
        return dst

    def getZeroBeliefbyRR(self,readRate):
        '''
        * description: based the readRate give zero belief, the high readRate low zero belief
        * input: readRate
        * output: zeroBelief
        '''
        zeroBelief = 0
        if readRate > 100:
            zeroBelief = 0.001
        elif readRate> 70:
            zeroBelief = 0.005
        elif readRate> 30:
            zeroBelief = 0.05
        elif readRate > 10:
            zeroBelief = 0.07
        else:
            zeroBelief = 0.1
        return zeroBelief

class rfidbotAntennaModelRawDataBase:
    def __init__(self):
        self.matrixWidth = None
        self.matrixHeight = None
        self.tagExpRawDataMatrix = None  #the raw data of the test record,tagExpRawDataMatrix[x][y][th]=readRate
        self.expMapResolution = None
        self.maxRange = None           #the max range of the robot move around the tag
        self.angularStep = None
        self.angularRange = None
        self.expRoateResolution = None #the robot roate maximum 180 degree in one position
                                       #the robot roate from 90 to 270 degree,no 270
        self.tagPosInMarix = None
        self.baseRoateAngule = None
        self.invalidRawData = []   #records the test positon which is invalid or robot not navigated
                                   #[idx,idy,th] is th = -1 for all ths
        self.expPowerLevel = None # [power1,power2,power3,...]

    def updateRoateResolution(self):
        if self.angularStep != 0:
            self.expRoateResolution = self.angularRange/self.angularStep


class rfidbotRFIDVectorModelBase:
    def __init__(self):
        self.AntennaInfo = None
        self.BeliefMapResolution = None 
        self.mapHeight = None
        self.mapWidth = None
        self.AntennaLoc = None       #the antenna center location in the map
        self.AntLocInBelMap = None   #the antenna center location belief map is the clo and row in matrix
        self.powerLevel = None # [power1,power2,power3,...]
        #self.powerWeight = None #[weight_power1,weight_power2,weight_power3,...]
        self.antennaVectorModel =  None #[x][y][rr80,rr110,rr140,rr170],rr80 is [min,max]
        self.zeroBel = 0.001
        self.BeliefMap = None       #[[powerlevel1][rr1][x][y],....
        self.BinaryMap = None # represent the belief by binary, if in a position has any reading is 1, othervise is 0
                              # [[powerlevel1][x][y],...
                              # TODO, to imporve the as BeliefMap, BinaryMap=[[powerlevel1][rr][x][y],...
                              #       Here rr is 0 or 1

    def diffBelief(self,modelReadRate, fieldReadRead):
        '''
        * description: based on the model read rate and field test one to give the belief. 
               1. the modelReadRate_min <= fieldReadRead <= modelReadRate_max, return 1
               2. 0 < fieldReadRead - modelReadRate_max < 10, return 0.8
               3. 10 < fieldReadRead - modelReadRate_max < 20, return 0.6
               4. 20 < fieldReadRead - modelReadRate_max < 30, return 0.4
               5. 30 < fieldReadRead - modelReadRate_max , return 0.2
               6. 0 < modelReadRate_min - fieldReadRead < 10, return 0.9
               7. 10 < modelReadRate_min - fieldReadRead < 20, return 0.8
               8. 20 < modelReadRate_min - fieldReadRead < 30, return 0.7
               9. 30 < modelReadRate_min - fieldReadRead , return 0.6
               ps, above setting need more test
        * input: modelReadRate, modeled read rate in spesfic powerlevel and position,
                            modelReadRate is [min, max]
                 fieldReadRead, test read rate in spesfic powerlevel and position
        '''
        modelRRMin = modelReadRate[0]
        modelRRMax = modelReadRate[1]
        
        if (modelRRMin <= fieldReadRead) and (fieldReadRead <= modelRRMax):
            return 1
            
        diffMax = fieldReadRead - modelRRMax
        if diffMax > 0:
            if diffMax < 10:
                return 0.8
            elif diffMax < 20:
                return 0.6
            elif diffMax < 30:
                return 0.4
            else:
                return 0.2
                
        diffMin = modelRRMin - fieldReadRead
        if diffMin > 0:
            if diffMin < 10:
                return 0.9
            elif diffMin < 20:
                return 0.8
            elif diffMin < 30:
                return 0.7
            else:
                return 0.6


    def updatePowerLevelWeight(self,readRateVector):
        '''
        * description: base on the pattern of the readRateVector, give different weight
                      1. [0,0,0,0]: this should be abandon before here
                      2. [0,0,0,1]: [0.05,0.05,0.05,0.85]
                      3. [0,0,1,1]: [0.05,0.05,0.5,0.4]
                      4. [0,1,1,1]: [0.05,0.4,0.3,0.25]
                      5. [1,1,1,1]: [0.4,0.3,0.2,0.1]
                      6, others, None (during this situation, there must be something abnormal, so 
                         should abandon this rearRateVector by give all the estimated position same
                         belief,like 1)
                      ps, here 1 means the read rate is not zero (should > 0)
        '''
        rr0 = readRateVector[0]
        rr1 = readRateVector[1]
        rr2 = readRateVector[2]
        rr3 = readRateVector[3]
        
        if (rr0 == 0) and (rr1 == 0) and (rr2 == 0) and (rr3 > 0) :
            #return [0.2,0.2,0.1,0.5]
            return [0.05,0.05,0.05,0.85]
            #return [0.05,0.05,0.05,0.1]
        elif (rr0 == 0) and (rr1 == 0) and (rr2 > 0) and (rr3 > 0) :
            #return [0.1,0.1,0.5,0.3]
            return [0.05,0.05,0.5,0.4]
            #return [0.05,0.05,0.2,0.1]
        elif (rr0 == 0) and (rr1 > 0) and (rr2 > 0) and (rr3 > 0) :
            #return [0.05,1.5,0.3,0.25]
            #return [0.1,0.4,0.3,0.2]
            return [0.05,0.4,0.3,0.25]
            #return [0.05,0.4,0.2,0.1]
        elif (rr0 > 0) and (rr1 > 0) and (rr2 > 0) and (rr3 > 0) :
            #return [4,0.3,0.2,0.1]
            return [0.4,0.3,0.2,0.1]
        else :
            return None

    def getBelinBeliefMap(self,PLId,readRate,mapIdRow,mapIdCol):
        '''
        * description, get the belief for sepsific powerlevel,readrate,estimated location(already 
                       convert to Row and column Id) by belief map,
        * input: PLId: the powerLevel Id
                 readRate: the read rate
                 mapIdRow: Row Id in belief map
                 mapIdCol: column Id in belief map
        * output: return belief
        '''

        if readRate > (len(self.BeliefMap[PLId]) - 1):
            readRate = len(self.BeliefMap[PLId]) - 1
        if readRate < 0 :
            readRate = 0

        belief = self.BeliefMap[PLId][readRate][mapIdRow][mapIdCol]

        return belief

    def getBelinBinaryMap(self,PLId,readRate,mapIdRow,mapIdCol): 
        '''
        * description, get the belief for sepsific powerlevel,readrate,estimated location(already 
                       convert to Row and column Id) by binary map,
                       if the gtRate in binary map is equal to input readRate, then the belief of
                       tag in esitmated position(represent by mapIdRow,mapIdCol) is 0.8, otherwise 
                       belief is 0.1 
        * input: PLId: the powerLevel Id
                 readRate: the read rate (0 or 1)
                 mapIdRow: Row Id in belief map
                 mapIdCol: column Id in belief map
        * output: return belief
        '''
        
        gtRate = self.BinaryMap[PLId][mapIdRow][mapIdCol]
        if gtRate == readRate:
            belief = 1 #0.8
        else:
            belief = 0.1
     
        return belief

    def mapPos2AntennaModelColRow(self,anPosX,anPosY,anPosTh,esLocX,esLocY):
        '''
        * description: map the position of estimated point into antenna model's column and row
        * input: anPosX,anPosY,anPosTh the antenna position in the map(x,y,th)
                 esLocX,esLocY  the esitmated point position
        * output: (mapIdCol,mapIdRow), the mapped column and row Id in antenna model metrix
        '''
        #mapping the map coordinate in to antenna coordinate
        #first rotate the coordinate to th
        esLox_1 = esLocX * math.cos(anPosTh) + esLocY * math.sin(anPosTh)
        esLoy_1 = esLocY * math.cos(anPosTh) - esLocX * math.sin(anPosTh)
        anPosx_1 = anPosX * math.cos(anPosTh) + anPosY * math.sin(anPosTh)
        anPosy_1 = anPosY * math.cos(anPosTh) - anPosX * math.sin(anPosTh)
        #in rotated coordinate, map the position into antenna coordinate
        newPosX = esLox_1 - (anPosx_1 - self.AntennaLoc[0])
        newPosY = (anPosy_1 + self.AntennaLoc[1]) - esLoy_1  
        mapIdCol = int(round(newPosX/self.BeliefMapResolution))
        mapIdRow = int(round(newPosY/self.BeliefMapResolution))
        return (mapIdCol,mapIdRow)
        
    def getBeliefMeasurement(self,anPosX,anPosY,anPosTh,esLocX,esLocY,readRateVector):
        '''
        * description: get the belief measurement by the antenna model in belief map
        * input: anPosX, the antenna position (x) in map
                 anPosY, the antenna position (y) in map
                 anPosTh, the antenna position (th) in map,unit is radians
                 esLocX, the estimated (candidate) position (x) in map
                 esLocY, the estimated (candidate) position (y) in map
                 readRateVector, the field read rate vector [rr80,rr110,rr140,rr170]
        '''
        (mapIdCol,mapIdRow) =  self.mapPos2AntennaModelColRow(anPosX,anPosY,anPosTh,esLocX,esLocY)

        #we give the position out of the model very low probability not 0
        if mapIdCol < 0 or mapIdCol > self.mapWidth - 1:
            return self.zeroBel
        if mapIdRow < 0 or mapIdRow > self.mapHeight - 1:
            return self.zeroBel

        powerLevelWeight = self.updatePowerLevelWeight(readRateVector)
        if powerLevelWeight == None:
            #when powerLevelWeight is none, the readRateVector pattern is wrong,so 
            #should abandon this rearRateVector by give all the estimated position same belief,like 1
            return 1
        measurement = 0
        for Id,modelReadRate in enumerate(self.antennaVectorModel[mapIdRow][mapIdCol]):
            #measurement = self.RFIDModel.BeliefMap[PLId][readrate][mapIdRow][mapIdCol]
            #measurement += powerLevelWeight[Id] * self.diffBelief(modelReadRate,readRateVector[Id])
            Bel =  self.getBelinBeliefMap(Id,readRateVector[Id],mapIdRow,mapIdCol)
            measurement += powerLevelWeight[Id] * Bel
             
        if measurement == 0:
            measurement = self.zeroBel
        return measurement

    def getZeroBel(self,readRateVector):
        '''
        * description: get teh zero belief based on the input vector, the idea is only high power
                       reading return high zero belief to reduce the weight of high power reading
        * input: 
                readRateVector, the field read rate vector [rr80,rr110,rr140,rr170]    
        '''

        rr0 = readRateVector[0]
        rr1 = readRateVector[1]
        rr2 = readRateVector[2]
        rr3 = readRateVector[3]

        if (rr0 == 0) and (rr1 == 0) and (rr2 == 0) and (rr3 > 0) :
            #return [0.05,0.05,0.05,0.85]
            return 0.01
        elif (rr0 == 0) and (rr1 == 0) and (rr2 > 0) and (rr3 > 0) :
            return 0.005
        elif (rr0 == 0) and (rr1 > 0) and (rr2 > 0) and (rr3 > 0) :
            return 0.002
        elif (rr0 > 0) and (rr1 > 0) and (rr2 > 0) and (rr3 > 0) :
            return 0.001
        else :
            return None

    def getBelMeasureinBinaryMap(self,anPosX,anPosY,anPosTh,esLocX,esLocY,readRateVector):
        '''
        * description: get the belief measurement by the antenna model in binary map
        * input: anPosX, the antenna position (x) in map
                 anPosY, the antenna position (y) in map
                 anPosTh, the antenna position (th) in map,unit is radians
                 esLocX, the estimated (candidate) position (x) in map
                 esLocY, the estimated (candidate) position (y) in map
                 readRateVector, the field read rate vector [rr80,rr110,rr140,rr170]
        '''
        (mapIdCol,mapIdRow) =  self.mapPos2AntennaModelColRow(anPosX,anPosY,anPosTh,esLocX,esLocY)

        #zeroBel = self.getZeroBel(readRateVector) #self.zeroBel
        zeroBel = self.zeroBel
        if zeroBel == None:
            return self.zeroBel
        #we give the position out of the model very low probability not 0
        if mapIdCol < 0 or mapIdCol > self.mapWidth - 1:
            return zeroBel
        if mapIdRow < 0 or mapIdRow > self.mapHeight - 1:
            return zeroBel

        powerLevelWeight = self.updatePowerLevelWeight(readRateVector)
        if powerLevelWeight == None:
            #when powerLevelWeight is none, the readRateVector pattern is wrong,so 
            #should abandon this rearRateVector by give all the estimated position same belief,like 1
            return 1
        measurement = 0
        for Id,modelReadRate in enumerate(self.antennaVectorModel[mapIdRow][mapIdCol]):
            Bel =  self.getBelinBinaryMap(Id,readRateVector[Id],mapIdRow,mapIdCol)
            measurement += powerLevelWeight[Id] * Bel
             
        if measurement == 0:
            measurement = zeroBel
        return measurement

    def getContourinBinaryMap(self,PLId,readRate,mapIdRow,mapIdCol): 
        '''
        * description, get contour value by binary map, if the esloc is in the antenna effect area
                       which get reading area, return 1, otherwise return 0
        * input: PLId: the powerLevel Id
                 readRate: the read rate (0 or 1)
                 mapIdRow: Row Id in belief map
                 mapIdCol: column Id in belief map
        * output: return belief
        '''
        
        gtRate = self.BinaryMap[PLId][mapIdRow][mapIdCol]
        if gtRate == readRate and readRate > 0:
            belief = 1 #0.8
        else:
            belief = 0
     
        return belief
        
    def getContourMeasureInBinaryMap(self,anPosX,anPosY,anPosTh,esLocX,esLocY,readRateVector):
        '''
        * description: get the contour measurement by the antenna model in binary map, use to 
                       support cluster filter
        * input: anPosX, the antenna position (x) in map
                 anPosY, the antenna position (y) in map
                 anPosTh, the antenna position (th) in map,unit is radians
                 esLocX, the estimated (candidate) position (x) in map
                 esLocY, the estimated (candidate) position (y) in map
                 readRateVector, the field read rate vector [rr80,rr110,rr140,rr170]
        '''
        (mapIdCol,mapIdRow) =  self.mapPos2AntennaModelColRow(anPosX,anPosY,anPosTh,esLocX,esLocY)
        zeroBel = 0

        #we give the position out of the model very low probability not 0
        if mapIdCol < 0 or mapIdCol > self.mapWidth - 1:
            return zeroBel
        if mapIdRow < 0 or mapIdRow > self.mapHeight - 1:
            return zeroBel

        powerLevelWeight = [1,1,1,1] # give the same weight for all power

        measurement = 0
        for Id,modelReadRate in enumerate(self.antennaVectorModel[mapIdRow][mapIdCol]):
            Bel =  self.getContourinBinaryMap(Id,readRateVector[Id],mapIdRow,mapIdCol)
            measurement += powerLevelWeight[Id] * Bel

        return measurement