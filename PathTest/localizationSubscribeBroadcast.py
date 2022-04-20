#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from rtabmap_ros.msg import MapGraph
import math
import numpy as np
from collections import deque

from matplotlib import pyplot as plt

posResult = []
oriResult = []

RAD_TO_DEGREES = 180 / math.pi
ANGLE_THRESHOLD = 10 # degrees
DISTANCE_THRESHOLD = .5 # meters

class Coord:
    def __init__(self, x, y, z, angle, qX, qY, qZ):
        self.x = x
        self.y = y
        self.z = z
        self.rad = angle
        self.degrees = angle * RAD_TO_DEGREES
        self.qX = qX
        self.qY = qY
        self.qZ = qZ

    def __str__(self):
        return '{'+ \
            f'"x":{self.x},"y":{self.y},"z":{self.z},'+ \
            f'"angle:"{self.degrees},"qX:"{self.qX},"qY"{self.qY},"qZ"{self.qZ}'+'}'



targetPos = None
path = deque()
turnFlag = True
recordFlag = False
followFlag = False


def callback(data):
    global path, targetPos, recordFlag, followFlag
    # rospy.loginfo('You got mail')
    # print(data.poses[-1].position)
    x = data.poses[-1].position.x
    y = data.poses[-1].position.y
    z = data.poses[-1].position.z
    # print(x*100, y*100, z*100)

    posResult.append(f'{x},{y},{z}')
    qX = data.poses[-1].orientation.x
    qY = data.poses[-1].orientation.y
    qZ = data.poses[-1].orientation.z
    w = data.poses[-1].orientation.w
    
    # Note still need to normalize quaternion before calculations
    angTheta = math.acos(w)*2

    # global currentPos
    currentPos = Coord(x,y,z,angTheta,qX,qY,qZ)

    if (recordFlag):
        if len(path) == 0:
            path.append(currentPos)

        dist = math.sqrt((currentPos.x-path[-1].x)**2+(currentPos.y-path[-1].y)**2)
        if dist > DISTANCE_THRESHOLD:
            print('point taken')
            path.append(currentPos)
    elif (followFlag):
        print(pointToPoint(currentPos, targetPos))


def pointToPoint(current:Coord, target:Coord):
    global targetPos, turnFlag, followFlag
    targetDeg = RAD_TO_DEGREES*math.atan2(targetPos.y-current.y, target.x-target.x)
    curDeg = current.degrees
    if current.qZ < 0:
        curDeg = 360 - curDeg

    if targetDeg < 0:
        targetDeg = 360 + targetDeg

    eucDist = math.sqrt((current.x-targetPos.x)**2+(current.x-targetPos.x)**2)
    if eucDist < .1:
        turnFlag = True
        if(len(path) == 0):
            followFlag = False
            return' END OF PATH'
        else:
            targetPos = path.pop()
            for point in path:
                print(point.strPos())
                print()
            return 'STOP'
        
        
    elif turnFlag and (10 < abs(curDeg - targetDeg)):
        if (targetDeg-curDeg > 0):
            return f'TURN LEFT{targetDeg:.2f} {curDeg:.2f}'
        else:
            return f'TURN RIGHT{targetDeg:.2f} {curDeg:.2f}'
    
    else:
        turnFlag = False 
        print('FORWARD TO:', eucDist)




def visualizePath():
    x = np.array()
    y = np.array()
    for point in path:
        x.append(point.x)
        y.append(point.y)
    plt.plot(x,y)
    plt.show()

def printPath(path):
    for point in path:
        print(f'{point.x, point.y}')


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

    print('Commands')
    print("  'r' : start recording")
    print("  'r' : stop recording")
    print("  's' : start pathfinding")
    print("  'q' : exit program")

    usrIn = input()

    while usrIn != 'q':
        if recordFlag == False and usrIn == 'r':
            print('Starting path recording')
            recordFlag = True
            path.clear()
        elif recordFlag and usrIn == 'r':
            print('Stopping path recording')
            printPath(path)
            recordFlag = False
        elif usrIn == 's':
            if len(path) > 0:
                print('Following path')
                followFlag = True
                targetPos = path.pop()
            else:
                print('Error: No Path to follow')
        usrIn = input()

