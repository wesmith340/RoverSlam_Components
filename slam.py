#!/usr/bin/env python
from distutils.dir_util import remove_tree
import rospy
from std_msgs.msg import String
from rtabmap_ros.msg import MapGraph
import math
import numpy as np
import roversim

from collections import deque

posResult = []
oriResult = []

RAD_TO_DEGREES = 180 / math.pi
PATH_THRESHOLD = .5 # 50 centimeters

class Coord:
    def __init__(self, x, y, z, angle, qX, qY, qZ):
        self.x = x
        self.y = y
        self.z = z
        self.rad = angle * -1
        self.degrees = angle * RAD_TO_DEGREES
        self.qX = qX
        self.qY = qY
        self.qZ = qZ

    def strPos(self):
        return '{'+f'"x":{self.x:.4f},"y":{self.y:.4f}'+'}'

    def __str__(self):
        return '{'+ \
            f'"x":{self.x},"y":{self.y},"z":{self.z},'+ \
            f'"angle:"{self.degrees},"qX:"{self.qX},"qY"{self.qY},"qZ"{self.qZ}'+'}'


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

    currentPos = Coord(x,y,z,angTheta,qX,qY,qZ)
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
    

    if (followFlag):
        targetDeg = RAD_TO_DEGREES*math.atan2(targetPos.y-currentPos.y, targetPos.x-currentPos.x)
        curDeg = currentPos.degrees
        if currentPos.qZ < 0:
            curDeg = 360 - curDeg

        if targetDeg < 0:
            targetDeg = 360 + targetDeg

        eucDist = math.sqrt((currentPos.x-targetPos.x)**2+(currentPos.y-targetPos.y)**2)
        if eucDist < .1:
            turnFlag = True
            if(len(path) == 0):
                followFlag = False
                print('END OF PATH')
            else:
                targetPos = path.pop()
                print('STOP')
                for point in path:
                    print(point.strPos())
                    print()
            
            
        elif (10 < abs(curDeg - targetDeg)):
            if (targetDeg-curDeg > 0):
                print('TURN LEFT', f'{targetDeg:.2f} {curDeg:.2f}')
            else:
                print('TURN RIGHT', f'{targetDeg:.2f} {curDeg:.2f}')
        
        else:
            turnFlag = False 
            print('FORWARD TO:', eucDist)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ekf_slam')
    rospy.Subscriber("/rtabmap/mapGraph", MapGraph, callback)
    # 

if __name__ == '__main__':
    listener()
    print("Valid inputs are")
    print("    'r': start recording")
    print("    's': stop recording")
    print("    'p': start pathfinding")
    print("    'q': end program")
    userIn = input()
    while (userIn != 'q'):
        if (userIn == 'r'):
            print('Starting record')
            path.clear()
            recordFlag = True
        if (userIn == 's'):
            print('Stopping record')
            recordFlag = False
            for point in path:
                print(point.strPos())
                print()
        if (userIn == 'p'):
            if len(path) > 0:
                print('Following path')
                followFlag = True
                targetPos = path.pop()
            else:
                print('ERROR: NO PATH TO FOLLOW')
        userIn = input()
    

    # with open('testPos.csv', 'w') as f:
    #     f.write("\n".join(posResult))
    # with open('testOri.csv', 'w') as f:
    #     f.write("\n".join(oriResult))