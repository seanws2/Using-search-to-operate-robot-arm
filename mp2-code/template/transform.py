
# transform.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
# 
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains the transform function that converts the robot arm map
to the maze.
"""
import copy
from arm import Arm
from maze import Maze
from search import *
from geometry import *
from const import *
from util import *

def transformToMaze(arm, goals, obstacles, window, granularity):
    """This function transforms the given 2D map to the maze in MP1.
    
        Args:
            arm (Arm): arm instance
            goals (list): [(x, y, r)] of goals
            obstacles (list): [(x, y, r)] of obstacles
            window (tuple): (width, height) of the window
            granularity (int): unit of increasing/decreasing degree for angles

        Return:
            Maze: the maze instance generated based on input arguments.

    """
    startAngle0 = arm.getArmAngle()[0]
    startAngle1 = arm.getArmAngle()[1]
    lim00 = arm.getArmLimit()[0][0]
    lim10 = arm.getArmLimit()[1][0]
    lim01 = arm.getArmLimit()[0][1]
    lim11 = arm.getArmLimit()[1][1]

    # build Maze skeleton
    maze = []
    i = 0
    j = 0
    while j < int(((abs(lim00- lim01) / granularity)) + 1):
        maze.append([])
        while i < int(((abs(lim10- lim11) / granularity)) + 1):
            maze[j].append(SPACE_CHAR)
            i += 1
        i = 0
        j+= 1

    # draw Maze
    for x in range(lim00, lim01 + 1):
        for y in range(lim10, lim11 + 1):
            #adjust arm angle
            angleSpot = angleToIdx([x, y], [lim00, lim10], granularity)
            arm.setArmAngle(idxToAngle([angleSpot[0], angleSpot[1]], [lim00, lim10], granularity))

            # variables used in conditional for
            armEnd = arm.getEnd()
            armPosDist = arm.getArmPosDist()
            checkMina = angleToIdx([startAngle0], [lim00], granularity)[0]
            checkMinb = angleToIdx([startAngle1], [lim10], granularity)[0]


            if angleSpot[0] == checkMina and angleSpot[1] == checkMinb: maze[angleSpot[0]][angleSpot[1]] = START_CHAR
            elif doesArmTipTouchGoals(armEnd, goals): maze[angleSpot[0]][angleSpot[1]] = OBJECTIVE_CHAR
            elif isArmWithinWindow(armPosDist, window) == False or doesArmTouchObjects(armPosDist, goals, True) == True or \
                    doesArmTouchObjects(armPosDist, obstacles, False) == True: maze[angleSpot[0]][angleSpot[1]] = WALL_CHAR
            #order maybe wrong -> check if arm touches objects first
    output = Maze(maze, [lim00, lim10], granularity)
    return output
