#!/usr/bin/env python3
'''
* File name: rfidbot_tags_debuger.py
* Description: provides the debug function for tag localization
* Author: Jian Zhang
* Create date: Jan/12/2016
* Modified date: Nov/10/2025 by Justin Palm
* Version 2.0 for ROS2 Humble
'''

import rclpy
from rclpy.node import Node
import numpy as np
from matplotlib import pyplot as plt
import pickle

# for debugger
from visualization_msgs.msg import MarkerArray, Marker
from rfidbot_tags_localization.libs.rfidbot_set_tags_location_marker import (
    setaNewMarker, setaSingleMarker,
    setaNewArrowMarker, setMarkerwithTransparent
)


'''
* class: RRRBDebuger
'''
class RRRBDebuger(Node):
    def __init__(self):
        super().__init__('rrrb_debuger')
        self.rawdata_pub = self.create_publisher(MarkerArray, 'epc_pos', 10)
        self.a_rawdata_item_pub = self.create_publisher(Marker, 'epc_center_weighted', 10)
        self.a_rawdata_item_pub_1 = self.create_publisher(Marker, 'raw_data_arrow', 10)

    '''
    * function name: visualRawDatainMap
    * description: using MarkerArray to show the rawdata in the map by estimate arrow
    * input: rawData, the raw data
    '''
    def visualRawDatainMap(self, rawData):
        markerArray = MarkerArray()
        for rData in rawData:
            posx = rData.antennaPose.pose.pose.position.x
            posy = rData.antennaPose.pose.pose.position.y
            posz = rData.antennaPose.pose.pose.position.z
            oriz = rData.antennaPose.pose.pose.orientation.z
            oriw = rData.antennaPose.pose.pose.orientation.w
            marker = setaNewMarker(posx, posy, oriz, oriw, 'g', 0.10, "/map")
            marker.header.stamp = self.get_clock().now().to_msg()
            markerArray.markers.append(marker)

        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
        self.get_logger().info("visualizing the position of rawdata in the map!")
        self.rawdata_pub.publish(markerArray)

    '''
    * function name: visualARawData
    * description: show a single rawdata item with an arrow marker
    * input: rawDataItem, the raw data item
    '''
    def visualARawData(self, rawDataItem):
        posx = rawDataItem.antennaPose.pose.pose.position.x
        posy = rawDataItem.antennaPose.pose.pose.position.y
        posz = rawDataItem.antennaPose.pose.pose.position.z
        oriz = rawDataItem.antennaPose.pose.pose.orientation.z
        oriw = rawDataItem.antennaPose.pose.pose.orientation.w
        marker = setaNewArrowMarker(posx, posy, oriz, oriw, 'r', 3.2, 0.5, 0.1, "/map")
        marker.header.stamp = self.get_clock().now().to_msg()
        self.a_rawdata_item_pub.publish(marker)

    '''
    * function name: normalizeBelData
    * description: normalize the belData, using the max bel value as normalizer
    * input: belData, the bel data
    * output: the normalized bel data, the max bel value is 1
    '''
    def normalizeBelData(self, belData):
        maxBel = 0
        minBel = float('Inf')
        for bel in belData:
            if maxBel < bel['bel']:
                maxBel = bel['bel']
            if minBel > bel['bel']:
                minBel = bel['bel']
        self.get_logger().warn(f'the max bel {maxBel}, min bel {minBel}')
        if maxBel != 0:
            for bel in belData:
                bel['bel'] /= maxBel
        return belData

    '''
    * function name: copyABelData
    * description: make a copy for belData
    * input: belData, the bel data
    * output: a copy of belData
    '''
    def copyABelData(self, belData):
        dupBelData = []
        for item in belData:
            dupBelData.append(dict([('esloc', item['esloc']), ('bel', item['bel'])]))
        return dupBelData

    '''
    * function name: visualBelinMap
    * description: using MarkerArray to show the bel of RRRlocalizer in the map,
                   the bel will be normalized first, use the normalized value as transparency
    * input: belData, the bel data
    '''
    def visualBelinMap(self, belData, color='g'):
        dupBelData = self.copyABelData(belData)
        norBelData = self.normalizeBelData(dupBelData)
        markerArray = MarkerArray()

        for bel in norBelData:
            posx = bel['esloc'][0]
            posy = bel['esloc'][1]
            trans = bel['bel']
            marker = setMarkerwithTransparent(posx, posy, color, 0.2, trans, "/map")
            marker.header.stamp = self.get_clock().now().to_msg()
            markerArray.markers.append(marker)

        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
        self.get_logger().info("visualizing the position of belData in the map!")
        self.rawdata_pub.publish(markerArray)

    def setRater(self, rate, rater):
        self.rateRatio = rate
        self.r = rater

    '''
    * function Name: waitForPeriod
    * description: system wait for a period
    * input: 
            idleSeconds: the wait period in seconds
    * return: None
    '''
    def waitForPeriod(self, idleSeconds):
        if idleSeconds is None:
            return
        for i in range(0, int(self.rateRatio * idleSeconds)):
            self.r.sleep()

    '''
    * function Name: showAntennaByImage
    * description: show an AntennaByImage
    * input: 
            rData: a record of rawData
    * return: None
    '''
    def showAntennaByImage(self, rData):
        antennaPose = rData.antennaPose
        posTh = self.getAntennaAngle(antennaPose)
        readrate = rData.readRate
        if readrate > (len(self.RFIDModel.BeliefMap) - 1):
            readrate = len(self.RFIDModel.BeliefMap) - 1
        if readrate < 0:
            readrate = 0
        self.get_logger().warn(f'the th is {posTh}, readrate is {readrate}')
        rotatedModel = self.RFIDModel.rotateReadRateModel(posTh, readrate)
        plt.subplot(121)
        plt.imshow(self.RFIDModel.BeliefMap[readrate])
        plt.subplot(122)
        plt.imshow(rotatedModel)
        plt.show()

    def visualBelin3DMap(self, belData, color='g'):
        '''
        * function name: visualBelin3DMap
        * description: using MarkerArray to show the bel of RRRlocalizer in the 3D map,
                       the bel will be normalized first, use the normalized value as transparency
        * input: belData, the bel data
        '''
        dupBelData = self.copyABelData(belData)
        norBelData = self.normalizeBelData(dupBelData)
        markerArray = MarkerArray()

        for bel in norBelData:
            posx = bel['esloc'][0]
            posy = bel['esloc'][1]
            posz = bel['esloc'][2]
            if bel['bel'] < 0.5:
                continue
            trans = bel['bel']
            marker = self.setMarkerwithTransparentIn3D(posx, posy, posz, color, 0.1, trans, "/map")
            marker.header.stamp = self.get_clock().now().to_msg()
            markerArray.markers.append(marker)

        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
        self.rawdata_pub.publish(markerArray)

    def setMarkerwithTransparentIn3D(self, PosX, PosY, PosZ, color, size, trans, frameId):
        marker = Marker()
        marker.header.frame_id = frameId
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        if color == 'r':
            marker.color.a = trans
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif color == 'g':
            marker.color.a = trans
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif color == 'y':
            marker.color.a = trans
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif color == 'b':
            marker.color.a = trans
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = PosX
        marker.pose.position.y = PosY
        marker.pose.position.z = PosZ
        return marker

    def visualARawDatain3Dmap(self, rawDataItem, color='r'):
        '''
        * description: using MarkerArray to show the rawdata in the 3D map by estimated arrow
        * input: rawDataItem, the raw data item
        '''
        marker = self.setA3DArrowMarker(rawDataItem.antennaPose, color, 0.5, 0.05, 0.05, "/map")
        marker.header.stamp = self.get_clock().now().to_msg()
        self.a_rawdata_item_pub.publish(marker)

    def visualARawDatain3Dmap_1(self, rawDataItem, color='r'):
        '''
        * description: using MarkerArray to show the rawdata in the 3D map by estimated arrow
        * input: rawDataItem, the raw data item
        '''
        marker = self.setA3DArrowMarker(rawDataItem.antennaPose, color, 0.5, 0.05, 0.05, "/map")
        marker.header.stamp = self.get_clock().now().to_msg()
        self.a_rawdata_item_pub_1.publish(marker)

    def setA3DArrowMarker(self, Arrowpose, color, size_x, size_y, size_z, frameId):
        marker = Marker()
        marker.header.frame_id = frameId
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = size_x
        marker.scale.y = size_y
        marker.scale.z = size_z
        if color == 'r':
            marker.color.a = 0.65
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif color == 'g':
            marker.color.a = 0.65
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif color == 'y':
            marker.color.a = 0.65
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif color == 'b':
            marker.color.a = 0.65
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        marker.pose.orientation = Arrowpose.pose.pose.orientation
        marker.pose.position = Arrowpose.pose.pose.position
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = RRRBDebuger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

