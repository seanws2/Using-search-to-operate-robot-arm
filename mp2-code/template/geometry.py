# geometry.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains geometry functions that relate with Part1 in MP2.
"""

import math
import numpy as np
from const import *

def computeCoordinate(start, length, angle):
    """Compute the end cooridinate based on the given start position, length and angle.

        Args:
            start (tuple): base of the arm link. (x-coordinate, y-coordinate)
            length (int): length of the arm link
            angle (int): degree of the arm link from x-axis to couter-clockwise

        Return:
            End position (int,int):of the arm link, (x-coordinate, y-coordinate)
    """
    rangle = math.radians(angle)
    newx = start[0] + int(length * math.cos(rangle))
    newy = start[1] - int(length * math.sin(rangle))
    return newx, newy

def doesArmTouchObjects(armPosDist, objects, isGoal=False):
    """Determine whether the given arm links touch any obstacle or goal

        Args:
            armPosDist (list): start and end position and padding distance of all arm links [(start, end, distance)]
            objects (list): x-, y- coordinate and radius of object (obstacles or goals) [(x, y, r)]
            isGoal (bool): True if the object is a goal and False if the object is an obstacle.
                           When the object is an obstacle, consider padding distance.
                           When the object is a goal, no need to consider padding distance.
        Return:
            True if touched. False if not.
    """
    #check obstacles or goals
    output = False
    for arm in armPosDist:
        start = arm[0]
        stop = arm[1]
        for object in objects:
            edgeDist = (start[1] - stop[1]) ** 2 + (start[0] - stop[0]) ** 2
            if edgeDist == 0:  distance = ((object[0] - start[0]) ** 2 + (object[1] - start[1]) ** 2) ** 0.5
            else:
                diff0 = object[0] - start[0]
                diff1 = stop[0] - start[0]
                diff3 = object[1] - start[1]
                diff4 = stop[1] - start[1]
                dot = ((diff1)*(diff0)) + ((diff4)*(diff3))
                result = (start[0] + (diff1)*max(0, min(1, dot / edgeDist)), start[1] + (diff4)*max(0, min(1, dot / edgeDist)))
                distance = ((object[0] - result[0]) ** 2 + (object[1] - result[1]) ** 2) ** 0.5
                #maybe go back and just add padding on see what happens
            if isGoal == False:
                #for obstacles so they are padded
                if distance <= object[2] + arm[2]: return True
            else:
                #for goals since padding is not needed
                if distance <= object[2]: return True
    return output


def doesArmTipTouchGoals(armEnd, goals):
    """Determine whether the given arm tick  goals

        Args:
            armEnd (tuple): the arm tick position, (x-coordinate, y-coordinate)
            goals (list): x-, y- coordinate and radius of goals [(x, y, r)]. There can be more than one goal.
        Return:
            True if arm tip touches any goal. False if not.
    """
    for goal in goals:
        d = ((armEnd[0] - goal[0])**2 + (armEnd[1] - goal[1])**2)**0.5
        if d > goal[2]: continue
        else: return True
    return False


def isArmWithinWindow(armPos, window):
    """Determine whether the given arm stays in the window

        Args:
            armPos (list): start and end positions of all arm links [(start, end)]
            window (tuple): (width, height) of the window

        Return:
            True if all parts are in the window. False if not.
    """
    for arm in armPos:
        if (arm[0][0] > window[0] or arm[0][0] < 0): return False
        if (arm[1][0] > window[0] or arm[1][0] < 0): return False
        if (arm[1][1] > window[1] or arm[1][1] < 0): return False
        if (arm[0][1] > window[1] or arm[0][1] < 0): return False

        else: continue

    return True


if __name__ == '__main__':
    computeCoordinateParameters = [((150, 190),100,20), ((150, 190),100,40), ((150, 190),100,60), ((150, 190),100,160)]
    resultComputeCoordinate = [(243, 156), (226, 126), (200, 104), (57, 156)]
    testRestuls = [computeCoordinate(start, length, angle) for start, length, angle in computeCoordinateParameters]
    assert testRestuls == resultComputeCoordinate

    testArmPosDists = [((100,100), (135, 110), 4), ((135, 110), (150, 150), 5)]
    testObstacles = [[(120, 100, 5)], [(110, 110, 20)], [(160, 160, 5)], [(130, 105, 10)]]
    resultDoesArmTouchObjects = [
        True, True, False, True, False, True, False, True,
        False, True, False, True, False, False, False, True
    ]

    testResults = []
    for testArmPosDist in testArmPosDists:
        for testObstacle in testObstacles:
            testResults.append(doesArmTouchObjects([testArmPosDist], testObstacle))
            # print(testArmPosDist)
            # print(doesArmTouchObjects([testArmPosDist], testObstacle))

    print("\n")
    for testArmPosDist in testArmPosDists:
        for testObstacle in testObstacles:
            testResults.append(doesArmTouchObjects([testArmPosDist], testObstacle, isGoal=True))
            # print(testArmPosDist)
            # print(doesArmTouchObjects([testArmPosDist], testObstacle, isGoal=True))

    assert resultDoesArmTouchObjects == testResults

    testArmEnds = [(100, 100), (95, 95), (90, 90)]
    testGoal = [(100, 100, 10)]
    resultDoesArmTouchGoals = [True, True, False]

    testResults = [doesArmTipTouchGoals(testArmEnd, testGoal) for testArmEnd in testArmEnds]
    assert resultDoesArmTouchGoals == testResults

    testArmPoss = [((100,100), (135, 110)), ((135, 110), (150, 150))]
    testWindows = [(160, 130), (130, 170), (200, 200)]
    resultIsArmWithinWindow = [True, False, True, False, False, True]
    testResults = []
    for testArmPos in testArmPoss:
        for testWindow in testWindows:
            testResults.append(isArmWithinWindow([testArmPos], testWindow))
    assert resultIsArmWithinWindow == testResults

    print("Test passed\n")
