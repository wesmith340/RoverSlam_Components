#!/usr/bin/env python
from distutils.dir_util import remove_tree
import math
from multiprocessing.connection import answer_challenge
from turtle import Vec2D
import numpy as np
import roversim
import random as rand
import pygame
import time
from collections import deque

posResult = []
oriResult = []

RAD_TO_DEGREES = 180 / math.pi
PATH_THRESHOLD = 5 # .5 # 50 centimeters
ONTOP_THRESHOLD = 5
OFFSET = 300


currentPos = None

startPos = None
targetPos = None

dumpPos = None
digPos = None

followFlag = False
recordFlag = False

turnFlag = True

path = deque()

curPath = deque()

def saveHome():
    # TODO
    # Implement save dump point and calculate dig point
    pass

def callback(data):
    global currentPos, targetPos, turnFlag, recordFlag, followFlag
    #print(data.poses[-1].position)
    x = data.poses[-1].position.x
    y = data.poses[-1].position.y
    z = data.poses[-1].position.z
    # print(x*100, y*100, z*100)

    # posResult.append(f'{x},{y},{z}')
    qX = data.poses[-1].orientation.x
    qY = data.poses[-1].orientation.y
    qZ = data.poses[-1].orientation.z
    w = data.poses[-1].orientation.w
    
    
    # Note still need to normalize quaternion before calculations
    angTheta = math.acos(w)*2
    s = math.sqrt(1-w*w) # normalized w < 1 guarenteed
    if s > 0.001: # avoid division by 0
        qX = qX / s
        qY = qY / s
        qZ = qZ / s

    roversim.ROVER.setPosition(roversim.Vector2(currentPos.y*100, currentPos.x*100))
    curRad = currentPos.rad
    if currentPos.qZ < 0:
        curRad = 2*math.pi - curRad
    roversim.ROVER.setRotation(curRad)

    roversim.draw()
    if recordFlag:
        if len(path) == 0:
            path.append(currentPos)
        
        lastPos = path[-1]

        if math.sqrt((x-lastPos.x)**2+(y-lastPos.y)**2) > PATH_THRESHOLD:
            roversim.ROVER.addToPath(roversim.Vector2(currentPos.y*100, currentPos.x*100))
            path.append(currentPos)
    
        

def nextMove(currentPos, angle, path):
    if(len(path) == 0):
        print('END OF PATH')
    else: 
        targetPos = path[-1]
        targetAngle = math.atan2(targetPos.y-currentPos.y, targetPos.x-currentPos.x)
        # curDeg = currentPos.degrees
        cAng = angle
        # if cAng < 0:
        #     cAng = 2*math.pi + cAng

        # if targetAngle < 0:
        #     targetAngle = 2*math.pi + targetAngle
        eucDist = math.sqrt((currentPos.x-targetPos.x)**2+(currentPos.y-targetPos.y)**2)
        if eucDist < ONTOP_THRESHOLD:
            targetPos = path.pop()
            print('STOP')
            # for point in path:
            #     print(point.strPos())
            #     print()
        elif (math.radians(10) < abs(targetAngle - cAng)):
            # if (targetAngle - cAng < 0):
            #     print('TURN LEFT', f'{targetAngle:.2f} {angle:.2f}')
            #     angle -= math.radians(5)
            #     if angle < -1*math.pi:
            #         angle = 2*math.pi + angle
            # else:
            #     print('TURN RIGHT', f'{targetAngle:.2f} {angle:.2f}')
            #     angle += math.radians(5)
            #     if angle > math.pi:
            #         angle = -2*math.pi + angle

            if targetAngle > cAng and targetAngle - cAng < math.pi or targetAngle < cAng and cAng - targetAngle > math.pi:
                print('TURN LEFT', f'{targetAngle:.2f} {angle:.2f}')

                angle += math.radians(5)
                angle = wrap(angle)
            else:
                print('TURN RIGHT', f'{targetAngle:.2f} {angle:.2f}')
                angle -= math.radians(5)
                angle = wrap(angle)

        
        else:
            x = currentPos.x + 5*math.cos(angle)
            y = currentPos.y + 5*math.sin(angle)
            currentPos = roversim.Vector2(x,y)
            print('FORWARD TO:', eucDist)
    
    return currentPos, angle

def wrap(angle):
    if angle > math.pi:
        angle = angle - 2*math.pi
        pass
    elif angle < -1*math.pi:
        angle = 2*math.pi + angle
        pass
    return angle

def randomPoint(point:roversim.Vector2):
    xOffset = rand.randrange(-1*roversim.WIDTH/2,roversim.WIDTH/2)
    yOffset = rand.randrange(-1*roversim.HEIGHT/2,roversim.HEIGHT/2)
    return roversim.ORIGIN + roversim.Vector2(xOffset, yOffset)

if __name__ == '__main__':
    running = True
    path = deque()

    roversim.ROVER.setPosition(randomPoint(roversim.ORIGIN))
    roversim.ROVER.setRotation(math.pi)
    roversim.draw()

    path.append(roversim.ORIGIN)
    roversim.ROVER.addToPath(roversim.ORIGIN)
    
    for i in range(10):
        nextPoint = randomPoint(path[-1])
        print(nextPoint.getTuple())
        path.append(nextPoint)
        roversim.ROVER.addToPath(nextPoint)
    print()
    for point in path:
        print(point.getTuple())
    

    curPos = roversim.ROVER.position
    angle = roversim.ROVER.rotation
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        roversim.screen.fill((255, 255, 255))
        roversim.ROVER.draw(roversim.screen)
        pygame.display.flip()

        curPos, angle = nextMove(curPos, angle, path)
        roversim.ROVER.setPosition(curPos)
        roversim.ROVER.setRotation(angle)
        time.sleep(.1)