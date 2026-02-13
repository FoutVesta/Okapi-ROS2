#!/usr/bin/env python3
'''
*File name: phase_differ_localizer.py
*Description: Based on the phase observation of RFID reader and the location of antenna where to collect the observation
*             use a probabilistic approach based on recursive Bayesian updating to localize a tag in 3d
*             The detail of the algorithm please refer to "A differential phase-based method for precise RFID localization"
*
*notes: 1. The pose in the raw data should be the pose of the reader antenna, phDiffTagLoclizer does not execute any 
           pose transform.
        2. The phase should follow the rule: thea=(2pi(2R/lamda)+thea_niose )mod 2pi. The original phase observation 
           from zebra FX7500 and RFD 8500 are thea=-1* (2pi(2R/lamda)+thea_niose )mod 2pi. phDiffTagLoclizer does not
           reverse the phase, the reader software should correct it.
        3. The phase must in radian, phDiffTagLoclizer does not execute any transform 
*Author: Jian zhang
*Modified by Justin Palm Nov/10/2025
*Version 2.0 for ROS2 Humble
'''

import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import importlib, types, sys
import os, pickle, math, time, itertools
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped
import tf2_ros
import tf2_geometry_msgs
import tf_transformations # replaces 'tf.transformations'

# ====== ABSOLUTE IMPORTS FOR ROS2 PACKAGE ======
from rfidbot_tags_localization.localizers.phase_differ_3d_strutured_obs import structedObservations
from rfidbot_tags_localization.localizers.rfidbot_tags_recursive_bayes_localize import rfidbotRBLocalizeATag
from rfidbot_tags_localization.libs.rfh_tags_localization_pose_shifter import rfhposeShifter
from rfidbot_tags_localization.libs.rfidbot_tags_debuger import RRRBDebuger

# ====== CONSTANTS ======
module_dir = os.path.dirname(os.path.abspath(__file__))
pardir = os.path.abspath(os.path.join(module_dir, os.pardir))
RFID3DModelPath = os.path.join(pardir, 'data', 'antenna3dModel_RFD8500.txt')
root_dir = os.path.abspath(os.path.join(pardir, os.pardir))

LOC_FLAG_NO_RAW_DATA = 0
LOC_FLAG_ONLY_INV = 1
LOC_FLAG_LOCALIZED = 2

antennaMaxRange = 3.4
c = 299792458.0
DEBUGET_SWITCH = True
CHANNEL_AMOUNT = 1
BEL_THES = 0.3
HYPOTHESIS_SAMPLE_STEP = 0.01
SEED_SAMPLE_STEP = 0.2
READER_THERM_NIOSE_STD = 0.1
HYPOTHESIS_SEED_BEL = 0.5
DEDUP_DIS = 0.01


class phDiffTagLoclizer(rfidbotRBLocalizeATag):
    '''
    * Class: phDiffTagLoclizer
    * Based on RF phase observation and recursive Bayesian updating to localize a tag in 3D.
    '''
    def __init__(self, node: Node):
        super().__init__(node)
        self.name = "3D tag localizer"
        self.logger = get_logger('phDiffTagLoclizer')
        self.RFID3DModel = None
        self.poseShifter = rfhposeShifter()
        self.debuger = RRRBDebuger()
        self.minRfidObDis = 0.01
        self.loadRFID3DModel()

    def _logwarn(self, msg, *args):
        self.logger.warn(msg % args if args else msg)

    def _loginfo(self, msg, *args):
        self.logger.info(msg % args if args else msg)

    # ===================
    # MODEL LOADING
    # ===================
    def loadRFID3DModel(self):
        if 'rospy' not in sys.modules:
            sys.modules['rospy'] = types.SimpleNamespace()
        try:
            if 'rfidbot_antenna_model_base' not in sys.modules:
                importlib.import_module('rfh_share_lib.rfidbot_antenna_model_base')
                sys.modules['rfidbot_antenna_model_base'] = sys.modules['rfh_share_lib.rfidbot_antenna_model_base']
        except Exception:
            pass

        candidate_paths = [
            RFID3DModelPath,
            os.path.join(root_dir, 'data', 'antenna3dModel_RFD8500.txt'),
        ]
        try:
            share_dir = get_package_share_directory('rfidbot_tags_localization')
            candidate_paths.append(os.path.join(share_dir, 'data', 'antenna3dModel_RFD8500.txt'))
        except Exception:
            pass

        chosen = next((p for p in candidate_paths if os.path.isfile(p)), None)
        if chosen is None:
            searched = "\n  - ".join(candidate_paths)
            raise FileNotFoundError(
                f"Antenna model file not found. Searched:\n  - {searched}\n"
                "Place the model file or pass parameter 'antenna_model_path'."
            )

        with open(chosen, 'rb') as f:
            self.RFID3DModel = pickle.load(f)
        self._logwarn("read antenna model from file %s", chosen)
        self._logwarn("info: %s", self.RFID3DModel.AntennaInfo)
        self._logwarn("Resolution: %.2f", self.RFID3DModel.BeliefMapResolution)
        self._logwarn("mapWidth: %d", self.RFID3DModel.mapWidth)
        self._logwarn("mapHeight: %d", self.RFID3DModel.mapHeight)
        self._logwarn("mapVertical: %d", self.RFID3DModel.mapVertical)
        self._logwarn("AntLocInBelMap: (%d,%d,%d)", *self.RFID3DModel.AntLocInBelMap)
        self._logwarn("AntennaLoc: (%.2f,%.2f,%.2f)", *self.RFID3DModel.AntennaLoc)

    # ===================
    # CORE FUNCTIONS
    # ===================
    def init3DHypothesisLoc(self, rawData):
        initLocSet = []
        MinX = rawData[0].antennaPose.pose.pose.position.x
        MinY = rawData[0].antennaPose.pose.pose.position.y
        MinZ = rawData[0].antennaPose.pose.pose.position.z
        MaxX, MaxY, MaxZ = MinX, MinY, MinZ
        for rawItem in rawData:
            px, py, pz = (rawItem.antennaPose.pose.pose.position.x,
                          rawItem.antennaPose.pose.pose.position.y,
                          rawItem.antennaPose.pose.pose.position.z)
            MinX, MaxX = min(MinX, px), max(MaxX, px)
            MinY, MaxY = min(MinY, py), max(MaxY, py)
            MinZ, MaxZ = min(MinZ, pz), max(MaxZ, pz)
        offsetRange = antennaMaxRange * 0.5
        MinX -= offsetRange; MaxX += offsetRange
        MinY -= offsetRange; MaxY += offsetRange
        MinZ -= offsetRange; MaxZ += offsetRange

        self._logwarn("the Maximum area x(%.4f,%.4f),y(%.4f,%.4f),z(%.4f,%.4f), with sample step:%.2f",
                      MinX, MaxX, MinY, MaxY, MinZ, MaxZ, SEED_SAMPLE_STEP)
        x = MinX
        while x < MaxX:
            x += SEED_SAMPLE_STEP
            y = MinY
            while y < MaxY:
                y += SEED_SAMPLE_STEP
                z = MinZ
                while z < MaxZ:
                    z += SEED_SAMPLE_STEP
                    initLocSet.append([x, y, z])
        return initLocSet

    def filterOutRawData(self, rawData):
        return [r for r in rawData if (r.antennaPose and r.antennaPose.pose and r.antennaPose.pose.pose)]

    def generateTf(self, rawItem):
        antennaPose = rawItem.antennaPose
        res = TransformStamped()
        res.header.stamp = rclpy.time.Time().to_msg()
        res.header.frame_id = "global"
        res.child_frame_id = "rfid_model"
        res.transform.translation = antennaPose.pose.pose.position
        res.transform.rotation = antennaPose.pose.pose.orientation
        transformer = TransformerROS()
        transformer.setTransform(res)
        try:
            transformer.lookupTransform("rfid_model", "global", rclpy.time.Time().to_msg())
        except Exception as e:
            self._logwarn("tf failed: %s", str(e))
            return None
        return transformer

    def getBelin3DModel(self, rawItem, esLoc, transformer):
        zeroBel = 1e-9
        gpoint = PointStamped()
        gpoint.point.x, gpoint.point.y, gpoint.point.z = esLoc
        gpoint.header.stamp = rclpy.time.Time().to_msg()
        gpoint.header.frame_id = "global"
        try:
            mpoint = transformer.transformPoint("rfid_model", gpoint)
        except Exception:
            return 1
        model_x = mpoint.point.y + self.RFID3DModel.AntennaLoc[0]
        model_y = mpoint.point.x + self.RFID3DModel.AntennaLoc[1]
        model_z = mpoint.point.z + self.RFID3DModel.AntennaLoc[2]
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
            self._logwarn("cannot find powerlevel %s", rawItem.powerLevel)
            return zeroBel
        Bel = self.RFID3DModel.BinaryMap[PLId][mapIdRow][mapIdCol][mapIdVel]
        return Bel if Bel != 0 else zeroBel

    def modN(self, p, n):
        return p - int(p / n) * n

    def gaussianModel(self, theoryPhase, practicalPhase, std):
        std = math.sin(0.1)
        deltaPh = self.modN((practicalPhase - theoryPhase), (2 * math.pi))
        tmp1 = math.exp(-1 * ((deltaPh) ** 2 / (2 * (std ** 2))))
        tmp2 = ((2 * math.pi) ** (-1 / 2)) * (1 / std)
        return tmp1 * tmp2

    def getPracticlePhaseDifference(self, rawItem, rawItemOld):
        newPhase = float(rawItem.phase)
        oldPhase = float(rawItemOld.phase)
        return (newPhase - oldPhase)

    def getTheoryPhaseDifference(self, rawItem, esLoc, rawItemOld, channelID):
        rp = rawItem.antennaPose.pose.pose.position
        ro = rawItemOld.antennaPose.pose.pose.position
        esx, esy, esz = esLoc
        if channelID != rawItem.channelID or channelID != rawItemOld.channelID:
            self._logwarn("channelID mismatch (%s,%s,%s)", channelID, rawItem.channelID, rawItemOld.channelID)
        NewDistance = math.sqrt((rp.x - esx) ** 2 + (rp.y - esy) ** 2 + (rp.z - esz) ** 2)
        OldDistance = math.sqrt((ro.x - esx) ** 2 + (ro.y - esy) ** 2 + (ro.z - esz) ** 2)
        channelFrequency = self.channelTable(channelID)
        tmp1 = 4 * math.pi * NewDistance / (c / channelFrequency)
        tmp2 = 4 * math.pi * OldDistance / (c / channelFrequency)
        NewDistancePhase = self.modN(tmp1, (2 * math.pi)) - math.pi
        OldDistancePhase = self.modN(tmp2, (2 * math.pi)) - math.pi
        return (NewDistancePhase - OldDistancePhase)

    def channelTable(self, channelID):
        tableZebra7500 = {str(i + 1): f for i, f in enumerate([
            915750, 915250, 903250, 926750, 926250, 904250, 927250, 920250, 919250, 909250,
            918750, 917750, 905250, 904750, 925250, 921750, 914750, 906750, 913750, 922250,
            911250, 911750, 903750, 908750, 905750, 912250, 906250, 917250, 914250, 907250,
            918250, 916250, 910250, 910750, 907750, 924750, 909750, 919750, 916750, 913250,
            923750, 908250, 925750, 912750, 924250, 921250, 920750, 922750, 902750, 923250
        ])}
        return tableZebra7500[str(channelID)] * 1000
        
    def projectARawItem2Map(self,rawItem,transformer):
        '''
        * description: this function is used for dubug, it project the belief of a raw data item
                       to map
        * input: rawItem
        '''
        for id,esLoc in enumerate(self.estimatedLocation): 
    
            meas = self.getBelin3DModel(rawItem,esLoc,transformer)
           
            self.antennaBelInmap[id]['bel'] = meas
        rospy.logwarn("project(%.3f,%.3f,%.3f) model to map",
                        rawItem.antennaPose.pose.pose.position.x,
                        rawItem.antennaPose.pose.pose.position.y,
                        rawItem.antennaPose.pose.pose.position.z)
        self.debuger.visualBelin3DMap(self.antennaBelInmap,'y')
        self.debuger.visualARawDatain3Dmap(rawItem)

    def belUpdate(self,id,rawItem,rawItemOld,channelID,std):
        '''
        * Description: override the belUpdate to support 3D
        * input: 
        *       id, the id of estimated posittion in the esLocBel list
                rawItem, current raw data Item
                rawItemOld, the old raw data item, 
                            under the antenna moving scenario, the raw data of the atnnena in the last position,
                            under the tag moving scenario, the raw data of the tag in the last position.
                channelID, the channel ID of the data, the current and old raw data must be observed in a same channel
                std, the std niosed phase that is distorted by therm noise.                
        '''
        #xy add
        
        
        #jian add: for improve loclization speed
        #if self.esLocBel[id]['bel'] < BEL_THES:
        #    return
        #meas = self.getBelin3DModel(rawItem,esLoc,transformer)
        #rospy.logwarn("position(%.3f,%.3f,%.3f)",
        #                rawItem.antennaPose.pose.pose.position.x,
        #                rawItem.antennaPose.pose.pose.position.y,
        #                rawItem.antennaPose.pose.pose.position.z)
        esLoc = self.esLocBel[id]['esloc'] #the estimated posittion in the map (x,y,z)
        TheoryPhaseDifference = self.getTheoryPhaseDifference(rawItem,esLoc,rawItemOld,channelID)
        
        PracticlePhaseDifference = self.getPracticlePhaseDifference(rawItem,rawItemOld)

        channelFrequency = self.channelTable(channelID)

        #debug
        #TheoryPhaseDifference=math.sin(TheoryPhaseDifference)
        #PracticlePhaseDifference=math.sin(PracticlePhaseDifference)
        #std=math.sin(std)


        MatchIndex = self.gaussianModel(TheoryPhaseDifference,PracticlePhaseDifference,std) # + 0.000000001 

        newBel = self.esLocBel[id]['bel'] + MatchIndex
        self.esLocBel[id]['bel'] = newBel
        
        newBel1 = self.esLocBel[id]['chBel'][int(channelID)-1] + MatchIndex
        self.esLocBel[id]['chBel'][int(channelID)-1]=newBel1
               
        # debug
        '''
        rospy.logwarn("data1_position(%.3f,%.3f,%.3f),data2_position(%.3f,%.3f,%.3f),theoryPhaseDif %s, channelID %s, rawPhase 1 and 2: %s and %s, estimateLocation %s, belief %s,fre %s,saveloc %s",
                        rawItem.antennaPose.pose.pose.position.x,
                        rawItem.antennaPose.pose.pose.position.y,
                        rawItem.antennaPose.pose.pose.position.z,
                        rawItemOld.antennaPose.pose.pose.position.x,
                        rawItemOld.antennaPose.pose.pose.position.y,
                        rawItemOld.antennaPose.pose.pose.position.z,
                        TheoryPhaseDifference,
                        channelID,
                        float(rawItem.phase),
                        float(rawItemOld.phase),
                        esLoc,
                        MatchIndex,
                        channelFrequency,
                        self.esLocBel[id]['esloc']
                        )
        '''       
        #rospy.logwarn("location %s, believe: %s ,currentGaussian %s", self.esLocBel[id]['esloc'],self.esLocBel[id]['bel'], GaussianMatchIndex)
        
    def normalizeBelByMaxValue(self, logFlag = True):
        ''' 
        * description: normalize self.esLocBel['bel'], use the max value as normalizer
        * input: None
        * output: None   
        '''
        maxBel = 0
        maxLoc = None
        minBel = 1000
        for bel in self.esLocBel:
            if maxBel < bel['bel']:
                maxBel = bel['bel']
                maxLoc = bel['esloc']
            if minBel > bel['bel']:
                minBel = bel['bel']
        if logFlag:
            rospy.logwarn("the max bel : %s, max pos:%s, with min bel: %s",maxBel,maxLoc,minBel)         
        if maxBel != 0:
            for bel in self.esLocBel:
                bel['bel'] /= maxBel


    def absoluteBelValue(self):
        ''' 
        * description: get abs(self.esLocBel['bel'])
        * input: None
        * output: None   
        '''
        believeMap=[];
        corordinateMap=[];
        for bel in self.esLocBel:
            believeMap.append(abs(bel['bel']))
            corordinateMap.append(bel['esloc'])
            bel['bel'] = abs(bel['bel']) #jian added
            
        return believeMap, corordinateMap

    def estimatePosByAverageThresBel(self,esLocBel,Thres):
        '''
        * description: give a estimated position by weight average the all the estimated position 
                       which their beilief greater than thresihold, the weight is their belief
        * input: esLocBel, all the estimated postion with belief
                 Thres, the belief thresihold
        * output: (posx, posy, posz) the estimated position
        '''
        belSum = 0
        posx = 0
        posy = 0
        posz = 0
        for esLoc in esLocBel:
            if esLoc['bel'] > Thres:
                posx += esLoc['bel'] * esLoc['esloc'][0]
                posy += esLoc['bel'] * esLoc['esloc'][1]
                posz += esLoc['bel'] * esLoc['esloc'][2]
                belSum += esLoc['bel']
                #rospy.logwarn("location %s, believe: %s", esLoc['esloc'],esLoc['bel'])
        if belSum != 0:
            posx = posx / belSum
            posy = posy / belSum
            posz = posz / belSum
            return (posx,posy,posz)
        else:
            return (None,None,None)

    def estimatePosBMaxBel(self,esLocBel):
        posx = 0
        posy = 0
        posz = 0
        maxBel =0
        for esLoc in esLocBel:
            if esLoc['bel'] > maxBel:
                posx = esLoc['esloc'][0]
                posy = esLoc['esloc'][1]
                posz = esLoc['esloc'][2]
                maxBel = esLoc['bel']
                #rospy.logwarn("location %s, believe: %s", esLoc['esloc'],esLoc['bel'])
        return (posx,posy,posz,maxBel)
        
        
    def initTf2ForAntennaPose(self):
        '''
        * Description: initial the tf for change camera pose to antenna pose
        * input: None
        * output: return tf_buffer,tf_listener 
        '''
        tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        return(tf_buffer,tf_listener)

    def tfCameraPose2AntennaPose(self,tf_buffer,rData):
        '''
        * Description: change the camera pose to antenna pose
        * input: rData, include the rawdata pose, which is camera pose
        * output: rData, which update the pose to antenna one 
        '''
        targetFrame = "RFD8500_antena"
        sourceframe = "ZED_center" 
        globalFrame = "map"
        #def shiftPose(self,sourceFrameId,targetFrameId,globalFrameId,pose):
        rData.antennaPose.pose =self.poseShifter.shiftPose(sourceframe,
                                                            targetFrame,
                                                            globalFrame,
                                                            rData.antennaPose)
     

    def standardVariation(self,rawData):
        '''
        * Description: calculate standard variation for datas within a given channel
        * input: rawData
        * output: std (in radian)
        '''
        
        phaselist=[]
        for index in rawData:
            phaseInRad = math.pi/180 * float(index.phase)
            phaselist.append(phaseInRad)
        #rospy.logwarn("phaselist %s",phaselist)
        #std = np.std(phaselist, dtype=np.float64)
        #rospy.logwarn("std %s",std)
        
        std=0.1
        return std

    def calAntennaDisof2Ob(self, obItem1, obItem2):
        '''
        * Description: claculate the distance of the antenna between two observations
        * Input: obItem1, obItem2
        * Output: distance, in meter
        '''

        x1=obItem1.antennaPose.pose.pose.position.x
        y1=obItem1.antennaPose.pose.pose.position.y
        z1=obItem1.antennaPose.pose.pose.position.z

        x2=obItem2.antennaPose.pose.pose.position.x
        y2=obItem2.antennaPose.pose.pose.position.y
        z2=obItem2.antennaPose.pose.pose.position.z

        dis = math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
        
        return(dis)
  

    def filterbyThermalNoise(self,channelList,thermalStd):
        '''
        * Description: filter the channelList(the item sorted by channel), 
        *              The phase with thermal noise (with standeard devariation: thermalStd)
        *              if there are several items that distance with each other is less than
        *              sprt(2)*(thermalStd/2pi)* wave length, we only leave one item.
        *              rightnow, we do not change the original pahse in the list
        * Input:  channelList, the items sorted by 50 channels
                  thermalStd, standard deviation in radian
        * Output: filteredChList, the new filtered channel list 
        '''
        filteredChList= []
        for i in range(50):
            filteredChList.append([])

        for i in range(len(channelList)):
            RawData1Ch = channelList[i]
            if len(RawData1Ch) < 1:
                continue
                
            channelID = RawData1Ch[0].channelID  
            channelFz = self.channelTable(channelID)
            waveLen = c / channelFz
            thermalNoiseDis = math.sqrt(2)*(thermalStd/(2*math.pi)) * waveLen
            filteredChList[i]. append(RawData1Ch[0])

            #in furture add with k-mean to find the ceneter
            for j in range(1,len(RawData1Ch)):
                minDis = 9999
                for k in range(len(filteredChList[i])):
                    dis = self.calAntennaDisof2Ob(RawData1Ch[j],filteredChList[i][k])
                    if minDis > dis:
                        minDis = dis
                if minDis>thermalNoiseDis:
                    filteredChList[i]. append(RawData1Ch[j])

        return (filteredChList)
        
    def isdiffPhValid(self, rawData1,rawData2,std):
        '''
        * Description: two valid observations should satisfy: 
        *              d > (abs(delta_th)-std)/4pi * wavelength
        *              d is the distacne between two antennas
        *              delta_th is the phase differentce of two observation
        * Input: rawData1,rawData2,std
        * Output: true if valid, otherwise, false
        '''
        if rawData1.channelID != rawData2.channelID:
            return False
        channelFz = self.channelTable(rawData1.channelID)
        wavelen = c / channelFz

        obAntennasDis = self.calAntennaDisof2Ob(rawData1,rawData2)
        phase1 =  float(rawData1.phase)
        phase2 =  float(rawData2.phase)
        phaseDis = 0.5*((abs(phase1 - phase2) - 2*std)/(2*math.pi) * wavelen)

        if obAntennasDis > phaseDis:
            return True
        else:
            rospy.logwarn("ob1(%.5f,%.5f,%.5f, %s),ob2(%.5f,%.5f,%.5f, %s) is invalid (%f,%f)",
                                rawData1.antennaPose.pose.pose.position.x,
                                rawData1.antennaPose.pose.pose.position.y,
                                rawData1.antennaPose.pose.pose.position.z,
                                rawData1.phase,
                                rawData2.antennaPose.pose.pose.position.x,
                                rawData2.antennaPose.pose.pose.position.y,
                                rawData2.antennaPose.pose.pose.position.z,
                                rawData2.phase,
                                obAntennasDis,phaseDis)
            return False
        
        
    def printBelMap(self):
        '''
        * Description: after the belif is updated draw the belif map, only 2d(x,y)
        * Input: None
        * Output: None
        '''
        xs=[]
        ys=[]
        zs=[]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        for bel in self.esLocBel:
            zs.append(bel['bel'])
            xs.append(bel['esloc'][0])
            ys.append(bel['esloc'][1])
            #rospy.logwarn("pos(%.3f,%.3f),bel:%.3f",bel['esloc'][0],bel['esloc'][1],bel['bel'])
            #plt.scatter(bel['esloc'][0], bel['esloc'][1],c='r',alpha=bel['bel'])
            
        ax.scatter(xs, ys, zs)   
#        ax.set_xlabel('X Label')
#        ax.set_ylabel('Y Label')
        #ax.set_zlabel('Z Label')

        plt.show()
        
    def belUpdate43DBayes(self,id,rawItem,transformer):
        '''
        * Description: override the belUpdate to support 3D bayes localizer
        * input: 
        *       id, the id of estimated posittion in the esLocBel list
                rawItem, raw data Item
                transformer
        '''
        #jian add: for improve loclization speed
        if self.esLocBel[id]['bel'] < BEL_THES:
#            rospy.logwarn("self.esLocBel[id]['bel'] %s",self.esLocBel[id]['bel'])
            return
        esLoc = self.esLocBel[id]['esloc']
        meas = self.getBelin3DModel(rawItem,esLoc,transformer)
        newBel = self.esLocBel[id]['bel'] + meas
        self.esLocBel[id]['bel'] = newBel
        
    def get2RawDataAntennaDis2(self,rawdataItem1,rawdataItem2):
        '''
        * Description: calculate the sqauce of distance between the observed poistion of two raw data items
        * Input: rawdataItem1
                 rawdataItem2
        * Output: the distance^2
        '''
        dis2 =  ((rawdataItem1.antennaPose.pose.pose.position.x - rawdataItem2.antennaPose.pose.pose.position.x)**2 +
                (rawdataItem1.antennaPose.pose.pose.position.y - rawdataItem2.antennaPose.pose.pose.position.y)**2 +
                (rawdataItem1.antennaPose.pose.pose.position.z - rawdataItem2.antennaPose.pose.pose.position.z)**2)
        return dis2

    def deduplicateRawdata(self, rawData, dedupDis):
        '''
        * Description: deduplicate the raw data a antenna position only take 1 record
                       when the distance of two rawdata is less than dedupDis, we think these two rawdata are same one
                       so, we delet one
        * Input: rawData, the raw data 
                 dedupDis, the distance of two rawdata is less than dedupDis, we think these two rawdata are same one
        * Output:deduRawdata, the data after deduplication
        '''
        if len(rawData) < 2:
            return rawData

        dedupDis2 = dedupDis**2
        deduRawdata=[]
        deduRawdata.append(rawData[0])
        
        for i in range(1,len(rawData)):
            rawdataItem = rawData[i]
            minDis2 = 9999
            for deduRwItem in deduRawdata:
                dis2 = self.get2RawDataAntennaDis2(deduRwItem,rawdataItem)
                if dis2 < minDis2:
                    minDis2 = dis2
            if minDis2 > dedupDis2: # we keep the rawdata which the min distance to existing rawdata > 1cm
                deduRawdata.append(rawdataItem)
        return deduRawdata
    
    def getHyPositionSeed(self,filteredRawData):
        '''
        * Description: Base on the rawdata to esitmate the Hypothesis positions for the tag.
                       Hypothesis positions are gained by the 3D recursive bayes localizer, 
                       then choose the positions with believe greater than a threshold as the Hypothesis position seed.
        * Input:  filteredRawData, the valida raw data
        * Output: returen the position seed. format [[x,y,z],[x,y,z],..., [x,y,z]]
        '''
        
        deduRawdata = self.deduplicateRawdata(filteredRawData,DEDUP_DIS)
        if len(deduRawdata) < 4:
            rospy.logwarn("valid data(%d) is too same",len(deduRawdata))
            return None
        #initial hypothesis locations and belief
        self.estimatedLocation = self.init3DHypothesisLoc(deduRawdata)
        self.initEsLocBel()
        
        #recursively updating
        i=0
        for rData in deduRawdata: 
            
            if DEBUGET_SWITCH : 
                self.debuger.visualARawDatain3Dmap(rData)
                #time.sleep(10)
                
            g2m_tf = self.generateTf(rData) #the transform of RFID model with given pose in global frame
            if g2m_tf == None:
                continue
            
            MaxBel = 0
            for id in range(0, len(self.esLocBel)): 
                #update the positon i (bel(i))
                self.belUpdate43DBayes(id,rData,g2m_tf)
                if MaxBel < self.esLocBel[id]['bel']:
                    MaxBel = self.esLocBel[id]['bel']
                    
            newEsLocBel = [] 
            #delete the lower belief to increase the speed 
            for id in range(0, len(self.esLocBel)): 
                if self.esLocBel[id]['bel'] > (MaxBel * BEL_THES - 0.001): #-0.001 in case maximum = 0
                    newEsLocBel.append(self.esLocBel[id])
    
            self.esLocBel = newEsLocBel
            rospy.logwarn("HyPositionSeed: recursively updating %d of %d,rawdata(%.3f,%.3f,%.3f),esLocBel's size:%d, ",
                            i,len(deduRawdata),
                            rData.antennaPose.pose.pose.position.x,
                            rData.antennaPose.pose.pose.position.y,
                            rData.antennaPose.pose.pose.position.z,
                            len(self.esLocBel))
            i += 1
            
            if DEBUGET_SWITCH : 
                self.debuger.visualARawDatain3Dmap(rData)
                #time.sleep(5)

        #normolize all the bel at the end
        self.normalizeBelByMaxValue()
        
        # we only select the bel >HYPOTHESIS_SEED_BEL as our seed
        seedEsLocBel = []
        seedLocSet = []
        for id in range(0, len(self.esLocBel)):  
                if self.esLocBel[id]['bel'] > HYPOTHESIS_SEED_BEL:
                    seedEsLocBel.append(self.esLocBel[id])
                    seedLocSet.append(self.esLocBel[id]['esloc'])

        rospy.logwarn("size of seedEsLocBel:%s, and seedLocSet size:%s", len(seedEsLocBel),len(seedLocSet))
            
        if DEBUGET_SWITCH : 
            self.esLocBel = seedEsLocBel
            rospy.logwarn("visualBelin3DMap")
            self.debuger.visualBelin3DMap(self.esLocBel) 
            #time.sleep(5)

        return seedLocSet

    def sampleHyPositionSeed(self,hyPositionSeed, sampleStep):
        '''
        * Description: sample the hypothesis position seed to generate find hypothesis positions
        * Input:
        * Output:
        '''
        sampledPosTulpe =[]
        offsetRange = SEED_SAMPLE_STEP * 0.5 #the seed grid is SEED_SAMPLE_STEP,  
        for pos in hyPositionSeed:
            MinX = pos[0] - offsetRange
            MaxX = pos[0] + offsetRange
            MinY = pos[1] - offsetRange
            MaxY = pos[1] + offsetRange
            MinZ = pos[2] - offsetRange 
            MaxZ = pos[2] + offsetRange
            if MinZ< 0:
                MinZ = 0
            x = MinX
            while (x < MaxX):
                x += sampleStep
                y = MinY
                while (y < MaxY):
                    y += sampleStep
                    z = MinZ
                    while (z < MaxZ):
                        z += sampleStep
                        sampledPosTulpe.append((x,y,z))

        unqiuePosTulpe = set(sampledPosTulpe)
        sampledPosList = []
        for pos in unqiuePosTulpe:
            sampledPosList.append([pos[0],pos[1],pos[2]])
        rospy.logwarn("size of sampledPosTulpe:%s, sampledPosList:%s", len(sampledPosTulpe),len(sampledPosList))
        return sampledPosList
        
    def localizesATag(self,tagEPC, rawData):  
        '''
        * Description: localize a tag by given EPC and related rawdata
        * input: tagEPC, the EPC of tag
                 rawData, the rawdata of a tag,
                          The phase in rawData must in radians,
                          The pose in rawData shoud be the pose of the observed antenna 
        * output: return, loacliazation flag:LOC_FLAG_NO_RAW_DATA (there are now raw data, localization failed)
                                            LOC_FLAG_ONLY_INV (There are only limited data, localization failed)
                                            LOC_FLAG_LOCALIZED (localization success)
                          posx,posy,poxz, tag location in map
        '''
       
        rospy.logwarn("localizing tag %s",tagEPC)

        if None == rawData:
            rospy.warn("the rawData is None!")
            return (LOC_FLAG_NO_RAW_DATA,None,None,None) 

        #initial the tf for change camera pose to antenna pose
        (tf_buffer,tf_listener) = self.initTf2ForAntennaPose()
        
        #filter the rawdata
        filteredRawData = self.filterOutRawData(rawData)
        #filteredRawData = self.deduplicateRawdata(filteredRawData,self.minRfidObDis)
        rospy.logwarn("orginal raw data size %s, valide raw data size %s", len(rawData), len(filteredRawData))            
        if len(filteredRawData)< 1:
            return (LOC_FLAG_ONLY_INV,None,None,None) 

        #estimated the seed hypothesis location by 3D recursive bayes localizer
        hyPositionSeed = self.getHyPositionSeed(filteredRawData)
        if hyPositionSeed == None:
            return (LOC_FLAG_ONLY_INV,None,None,None) 
        
        self.estimatedLocation = self.sampleHyPositionSeed(hyPositionSeed,HYPOTHESIS_SAMPLE_STEP)  
        #initial hypothesis locations and belief
        #self.estimatedLocation = self.init3DHypothesisLoc(filteredRawData)
        self.initEsLocBel()

        #TODO: the std should be configed by the paramter
        std = READER_THERM_NIOSE_STD  
       
        #create the structed observation for differential phase localization
        structedObs = structedObservations()
        structedObs.setRawData(tagEPC,filteredRawData)
        structedObsData = structedObs.getNorObs()

        #estimating the tag location by observations between two pose of same antenna
        for obsAnt in structedObsData:    # loop all observations for all antennas
            rospy.logwarn("start update observation from antanna: %s",obsAnt["antennaID"])
            
            #get the combination of all obseravtons for the antenna, it combine two observations in defferen pose
            poseNum = len(obsAnt["obs"]) # get the pose number of the antenna
            posePairs=itertools.combinations(range(poseNum),2)  #get all the combinations of all poses
            posePairs = list(posePairs)
            pairNum = 0
       
            for posePair in posePairs:
                oldPoseObs = obsAnt["obs"][posePair[0]]["obs"]
                newPoseObs = obsAnt["obs"][posePair[1]]["obs"]
                pairNum +=1

                for i in range(CHANNEL_AMOUNT): # loop 50 channels 
                    oldPoseAChObs = oldPoseObs[i]["obs"] 
                    newPoseAChObs = newPoseObs[i]["obs"]
                    if len(oldPoseAChObs) != len(newPoseAChObs):  #the obs bwteen two poses in a same channel must own same number
                        rospy.logwarn("the channels,%d,with (%d,%d) is not equal",i,len(oldPoseAChObs), len(newPoseAChObs))
                        continue
                    for j in range(len(oldPoseAChObs)):
                        rDataOld = oldPoseAChObs[j]
                        rDataNew = newPoseAChObs[j]
                        
#                        dis = self.calAntennaDisof2Ob(rDataOld,rDataNew) 
#                        if (dis > 0.20) or (dis < 0.10):
#                            break
                            
                        if ((rDataOld.antennaID != rDataNew.antennaID) or 
                            (rDataOld == rDataNew) or 
                            (rDataOld.channelID != rDataNew.channelID)):
                            rospy.logwarn("old(an:%s,ch:%s), vs new(an:%s,ch:%s)",
                                            rDataOld.antennaID,rDataOld.channelID,
                                            rDataNew.antennaID,rDataNew.channelID)
                            continue
                            
                        channelNumber = rDataNew.channelID
                        #for id,esLoc in enumerate(self.estimatedLocation): 
                        
                        MaxBel = 0
                        for id in range(0, len(self.esLocBel)): 
                            #update the positon i (bel(i))
                            self.belUpdate(id,rDataNew,rDataOld,channelNumber,std)
                            if MaxBel < self.esLocBel[id]['bel']:
                                MaxBel= self.esLocBel[id]['bel']
                              
                        newEsLocBel = []   
                        for id in range(0, len(self.esLocBel)): 
                            #delete the lower belief to increase the speed 
                            if self.esLocBel[id]['bel'] > (MaxBel* BEL_THES - 0.001): # -0.001 in case maximum = 0
                                newEsLocBel.append(self.esLocBel[id])
                                
                        self.esLocBel = newEsLocBel
                        
                        rospy.logwarn("updated obs(%d in %d) in channel(%s) for pos(%.3f,%.3f,%.3f),pos(%.3f,%.3f,%.3f),with esLocBel size(%d) ",
                            pairNum, len(posePairs),rDataNew.channelID,
                            obsAnt["obs"][posePair[0]]["antennaPose"].pose.pose.position.x,
                            obsAnt["obs"][posePair[0]]["antennaPose"].pose.pose.position.y,
                            obsAnt["obs"][posePair[0]]["antennaPose"].pose.pose.position.z,
                            obsAnt["obs"][posePair[1]]["antennaPose"].pose.pose.position.x,
                            obsAnt["obs"][posePair[1]]["antennaPose"].pose.pose.position.y,
                            obsAnt["obs"][posePair[1]]["antennaPose"].pose.pose.position.z, 
                            len(self.esLocBel))
                    
                        
        savefile={}
        [believeMap, corordinateMap] = self.absoluteBelValue()
        savefile['believe'] = believeMap
        savefile['position'] = corordinateMap
        savefile['rawOutput'] = self.esLocBel
        
#        rospy.logwarn("saving map")
#        savename= '/home/rfid/map_new.mat'
#        pyio.savemat(savename,savefile)                   

        rospy.logwarn("localize %s is done!",tagEPC)
#        Thres = 0.2
#        (posx,posy,posz) = self.estimatePosByAverageThresBel(self.esLocBel,Thres) 
#        rospy.logwarn("estimate pose by average: x=%s,y=%s,z=%s",posx,posy,posz)
        (posx,posy,posz,maxBel) = self.estimatePosBMaxBel(self.esLocBel)
        rospy.logwarn(" max belief(%.4f) estimated pose: [x=%.3f,y=%.3f,z=%.3f]",
                                            float(maxBel),float(posx),float(posy),float(posz))
        
#        self.normalizeBelByMaxValue()
#        self.printBelMap()
        return (LOC_FLAG_LOCALIZED,posx,posy,posz)  
        
