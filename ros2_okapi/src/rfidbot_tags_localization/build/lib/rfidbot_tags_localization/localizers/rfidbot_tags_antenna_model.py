#!/usr/bin/env python3
'''
* File name: rfidbot_tags_antenna_model.py
* Description: Provides the antenna model for tag localization
* Author: Jian Zhang
* Original date: Oct/7/2015
* Modified Date: Nov/10/2025 by Justin Palm
* Version 2.0 for ROS2 Humble
'''

import math
from math import pi
from PIL import Image
import numpy as np
from matplotlib import pyplot as plt
import cv2

import rclpy
from rclpy.logging import get_logger

# ROS message type still included (useful if you visualize via /map or /grid)
from nav_msgs.msg import OccupancyGrid

antennaMaxRange = 3.2


class rfidbotRFIDModel:
    def __init__(self):
        self.logger = get_logger('rfidbotRFIDModel')

        self.AntennaInfo = None
        self.BeliefMapResolution = None
        self.BeliefMap = None
        self.mapHeight = None
        self.mapWidth = None
        self.AntennaLoc = None       # antenna center location in the map (x, y)
        self.AntLocInBelMap = None   # same location in matrix indices (row, col)
        self.antennaLevelBel = [
            [0, 20], [1, 15], [1.5, 10], [2, 5]
        ]  # [[center, centerbel], [level1, bel1], ...]
        self.zeroBel = 0.001

    # -----------------------------------------------
    # Geometry generation and conversion
    # -----------------------------------------------
    def generateAntennaModel(self, x, y, z, w):
        """Generates antenna model outline in map coordinates."""
        sin_theta = 2 * z * w
        original_point = [
            [3.01, 0], [2.87, 0.5236], [2.74, 0.7854], [2.56, 1.0472],
            [1.34, 1.57], [1.03, 3.14], [1.34, -1.57],
            [2.56, -1.0472], [2.74, -0.7854], [2.87, -0.5236]
        ]
        border = []
        for i, pt in enumerate(original_point):
            r = pt[0]
            theta = pt[1] + math.asin(sin_theta)
            x_now = r * math.cos(theta) + x
            y_now = r * math.sin(theta) + y
            border.append([x_now, y_now])
        return border

    def mapModel2PixelModel(self, AntennaModel):
        """Convert antenna map coordinates into pixel space."""
        ModelOutline = []
        for point in AntennaModel:
            posx = max((point[0]) / self.BeliefMapResolution, 0)
            posy = max((point[1]) / self.BeliefMapResolution, 0)
            ModelOutline.append([np.uint32(posx), np.uint32(posy)])
        return ModelOutline

    def drawPolygonOutline(self, polygon, ImgData):
        """Draws a polygon outline on the given image array."""
        pts = np.array(polygon, np.int32).reshape((-1, 1, 2))
        cv2.polylines(ImgData, [pts], True, 1)

    def fillPolygon(self, point_array, antennM, AntennMCenter):
        """Fills polygon interior."""
        x_vals = [p[0] for p in antennM]
        y_vals = [p[1] for p in antennM]
        x_min, x_max = min(x_vals), max(x_vals)
        y_min, y_max = min(y_vals), max(y_vals)

        fill_cor = []
        for i in range(x_min, x_max + 1):
            fill_cor.append([])
            for j in range(y_min, y_max + 1):
                if point_array[j][i] == 1:
                    fill_cor[i - x_min].append(j)

        for i in range(len(fill_cor)):
            if len(fill_cor[i]) != 0:
                for k in range(fill_cor[i][0], fill_cor[i][-1]):
                    point_array[k][i + x_min] = 1
        return point_array

    def calDistance(self, X1, Y1, X2, Y2):
        """Calculate squared Euclidean distance between two points."""
        disX = X1 - X2
        disY = Y1 - Y2
        return disX * disX + disY * disY

    # -----------------------------------------------
    # Belief setting
    # -----------------------------------------------
    def setModelBel(self, ImgData):
        """Set belief levels for pixels based on distance thresholds."""
        centerX = int(self.AntennaLoc[1] / self.BeliefMapResolution)
        centerY = int(self.AntennaLoc[0] / self.BeliefMapResolution)

        t1 = (self.antennaLevelBel[1][0] / self.BeliefMapResolution) ** 2
        t2 = (self.antennaLevelBel[2][0] / self.BeliefMapResolution) ** 2
        t3 = (self.antennaLevelBel[3][0] / self.BeliefMapResolution) ** 2

        for i in range(0, self.mapHeight):
            for j in range(centerY, self.mapWidth):
                if ImgData[i][j] > 0:
                    dist = self.calDistance(centerX, centerY, i, j)
                    if dist < t1:
                        ImgData[i][j] = self.antennaLevelBel[1][1]
                    elif dist < t2:
                        ImgData[i][j] = self.antennaLevelBel[2][1]
                    elif dist < t3:
                        ImgData[i][j] = self.antennaLevelBel[3][1]
        ImgData[centerX][centerY] = self.antennaLevelBel[0][1]
        return ImgData

    # -----------------------------------------------
    # Model generation and rotation
    # -----------------------------------------------
    def generateRFIDModelMap(self):
        """Generate a belief map model for the antenna footprint."""
        self.mapHeight = int(antennaMaxRange * 2 / self.BeliefMapResolution)
        self.mapWidth = int(antennaMaxRange * 2 / self.BeliefMapResolution)
        self.AntennaLoc = [antennaMaxRange, antennaMaxRange]
        self.AntLocInBelMap = [
            int(self.AntennaLoc[0] / self.BeliefMapResolution),
            int(self.AntennaLoc[1] / self.BeliefMapResolution)
        ]

        tmpImgData = np.zeros((self.mapHeight, self.mapWidth), np.uint8)

        polygon = self.generateAntennaModel(
            self.AntennaLoc[0], self.AntennaLoc[1], 0, 1
        )
        antennM = self.mapModel2PixelModel(polygon)
        self.drawPolygonOutline(antennM, tmpImgData)
        tmpImgData = self.fillPolygon(tmpImgData, antennM, self.AntennaLoc)
        tmpImgData = self.setModelBel(tmpImgData)
        self.BeliefMap = tmpImgData
        return tmpImgData

    def rotateModel(self, rotDegree):
        """Rotate the entire belief map by a specified angle."""
        rows, cols = self.mapHeight, self.mapWidth
        rotateCenterX, rotateCenterY = self.AntLocInBelMap
        M = cv2.getRotationMatrix2D((rotateCenterX, rotateCenterY), rotDegree, 1)
        dst = cv2.warpAffine(self.BeliefMap, M, (cols, rows))
        return dst

    def rotateReadRateModel(self, rotDegree, readRate):
        """Rotate a single-layer (per-power-level) read rate model."""
        rows, cols = self.mapHeight, self.mapWidth
        rotateCenterX, rotateCenterY = self.AntLocInBelMap
        M = cv2.getRotationMatrix2D((rotateCenterX, rotateCenterY), rotDegree, 1)
        dst = cv2.warpAffine(self.BeliefMap[readRate], M, (cols, rows))
        return dst

