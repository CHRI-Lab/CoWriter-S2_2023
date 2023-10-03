#!/usr/bin/env python
"""
Helper functions for the robot to write
"""

import math

def scaleAndFlipPoints(matrix):
    #? windows where the robot can move its arms
    # rangeOfPossibleY = [-0.2, 0]
    # rangeOfPossibleZ = [0.0, 0.15]
    # posRefRobot = 0.3
    rangeOfPossibleY = [0, 0.1]
    rangeOfPossibleZ = [0.0, 0.09]
    posRefRobot = 0.3


    # print(f'[scaleAndFlipPoints] matrix : {matrix}')
    vectY = [m[1] for m in matrix]
    vectZ = [m[2] for m in matrix]
    meanY = sum(vectY)/float(len(vectY))
    meanZ = sum(vectZ)/float(len(vectZ))
    
    #? flip Y
    # vectY = [i - 2*(i - meanY) for i in vectY]
    vectY = [i - 0.3 for i in vectY]

    #? center y & z
    maxY = float(max(vectY))
    minZ = float(min(vectZ))

    vectY = [i - maxY for i in vectY]
    vectZ = [i - minZ for i in vectZ]

    #? scale up y
    scaleFactorY = abs(max(rangeOfPossibleY) - min(rangeOfPossibleY))/abs(float(max(vectY)) - float(min(vectY)))
    vectY = [i*scaleFactorY-0.02 for i in vectY]
    # print(f'scaleFactorY = {scaleFactorY}')

    #? scale up Z
    scaleFactorZ = abs(max(rangeOfPossibleZ) - min(rangeOfPossibleZ))/abs(float(max(vectZ)) - float(min(vectZ)))
    vectZ = [i*scaleFactorZ+posRefRobot for i in vectZ]
    # print(f'scaleFactorZ = {scaleFactorZ}')

    #? compute X, the length of the arm should be approximately constant depending on z, y -> need to find x
    z0 = 0.45
    y0 = -0.12
    r = 0.2
    # for i, z in enumerate(vectZ):
    #     print(f'z = {z} | -math.pow((z-z0), 2) = {-math.pow((z-z0), 2)} || y = {vectY[i]} | - math.pow((vectY[i]-y0), 2) = {- math.pow((vectY[i]-y0), 2)} | r*r = {r*r} || sum = {-math.pow((z-z0), 2) - math.pow((vectY[i]-y0), 2) + r*r}')

    vectX = [math.sqrt(-math.pow((z-z0), 2) - math.pow((vectY[i]-y0), 2) + r*r) for i, z in enumerate(vectZ)]
    # vectX = [0.1 if i == 0 else 0.15 for i, z in enumerate(vectZ)]
    # vectX = [0.13 for i, z in enumerate(vectZ)]


    points = []
    for i, y in enumerate(vectY):
        points.append([vectX[i], y, vectZ[i], matrix[0][3], 0, 0])

    return points


def gapBetweenPoints(pointa, pointb):
    return math.sqrt(math.pow((pointa[0] - pointb[0]), 2) + math.pow((pointa[1] - pointb[1]), 2))


