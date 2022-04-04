#!/usr/bin/env python
from tracemalloc import start
import rospy
from std_msgs.msg import String
from rtabmap_ros.msg import MapGraph
import math
import time
from controls import NavType, ControlType

from collections import deque

RAD_TO_DEGREES = 180 / math.pi
PATH_THRESHOLD = .5 # 50 centimeters
DIG_DISTANCE = 20 # meters
DUMP_TIME = 10 # seconds
DIG_TIME = 10 # seconds

class Coord:
    def __init__(self, x, y, z, angle, qX, qY, qZ):
        self.x = x
        self.y = y
        self.z = z
        self.rads = angle
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


class Slam:
    
    def __init__(self):
        self.home = None
        self.digSite = None
        self.currentPos = None
        self.curPath = deque()
        self.recordedPath = deque()
        self.following = False
        self.turnFlag = True
        self.control = ControlType.NOTHING

    def saveHome(self):
        if self.currentPos is not None:
            self.home = self.currentPos
            angle = self.home.rads
            if self.home.qZ < 0:
                angle = 2*math.pi - angle
            
            angle += math.pi
            x = self.home.x + math.cos(angle)*20
            y = self.home.y + math.sin(angle)*20
            self.digSite = Coord(x, y,0,0,0,0,0)

            self.curPath.append(self.digSite)
            self.curPath.append(self.home)
            print(self.home)
            print(self.digSite)
            return True
        else:
            print('Ople couldnt find home')
            return False

    
    def callback(self, data):
        x = data.poses[-1].position.x
        y = data.poses[-1].position.y
        z = data.poses[-1].position.z

        qX = data.poses[-1].orientation.x
        qY = data.poses[-1].orientation.y
        qZ = data.poses[-1].orientation.z
        w = data.poses[-1].orientation.w
        
        # Note still need to normalize quaternion before calculations
        angTheta = math.acos(w)*2
        s = math.sqrt(1-w*w)
        if s > 0.001: # avoid division by 0
            qX = qX / s
            qY = qY / s
            qZ = qZ / s

        self.currentPos = Coord(x,y,z,angTheta,qX,qY,qZ)
        if self.following:
            if len(self.curPath) == 0:
                self.curPath.append(self.currentPos)
            
            lastPos = self.curPath[-1]

            if math.sqrt((x-lastPos.x)**2+(y-lastPos.y)**2) > PATH_THRESHOLD:
                self.curPath.append(self.currentPos)
        

        if (len(self.recordedPath) > 0):
            tarPos = self.recordedPath[-1]
            targetDeg = RAD_TO_DEGREES*math.atan2(
                tarPos.y-self.currentPos.y, 
                tarPos.x-self.currentPos.x
                )
            curDeg = self.currentPos.degrees
            if self.currentPos.qZ < 0:
                curDeg = 360 - curDeg

            if targetDeg < 0:
                targetDeg = 360 + targetDeg

            eucDist = math.sqrt((self.currentPos.x-tarPos.x)**2+(self.currentPos.y-tarPos.y)**2)
            if eucDist < .1:
                self.turnFlag = True
                self.recordedPath.pop()
                control = ControlType.STOP
                
                
            elif self.turnFlag and (10 < abs(curDeg - targetDeg)):
                if (targetDeg-curDeg > 0):
                    # print('TURN LEFT', f'{targetDeg:.2f} {curDeg:.2f}')
                    control = ControlType.LEFT
                else:
                    # print('TURN RIGHT', f'{targetDeg:.2f} {curDeg:.2f}')
                    control = ControlType.RIGHT
            
            else:
                self.turnFlag = False 
                # print('FORWARD TO:', eucDist)
                control = ControlType.FORWARD
            print(control)
            self.control = control

    def initPath(self):
        self.recordedPath = self.curPath.copy()
        self.curPath.clear()
        return NavType.FOLLOW_PATH

slam = Slam()


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ekf_slam')
    rospy.Subscriber("/rtabmap/mapGraph", MapGraph, slam.callback)

def followPath(digDumpFlag):
    control = slam.control
    navType = NavType.FOLLOW_PATH
    if control == ControlType.NOTHING:
        if digDumpFlag == ControlType.DIGGING:
            control = digDumpFlag
            navType = NavType.NOTHING
        else:
            navType = NavType.FIND_QR
            control = ControlType.STOP

    # elif objDetection.control != ControlType.NOTHING:
    #     control = objDetection.control
    return control, navType
    

def findQR(digDumpFlag):
    # TODO
    # Implement find QR code procedures
    time.sleep(5)
    # if QR code found and oriented correctly
    if digDumpFlag == ControlType.DIGGING: # Should only be true once
        if slam.saveHome():
            navType = slam.initPath()
            control = ControlType.STOP
    else: 
        navType = NavType.NOTHING
        control = ControlType.DUMPING

    return control, navType
    

def dig():
    pass

def dump():
    pass

if __name__ == '__main__':
    listener()


    curControl = ControlType.NOTHING
    curNavType = NavType.FIND_QR
    digDumpFlag = ControlType.DIGGING
    startTime = None
    running = True
    while running:
        try:
            if curNavType == NavType.FIND_QR:
                control, curNavType = findQR(digDumpFlag)

            elif curNavType == NavType.FOLLOW_PATH:
                control, curNavType = followPath(digDumpFlag)

            elif curControl == ControlType.DIGGING:
                time.sleep(DIG_TIME)
                digDumpFlag = ControlType.DUMPING
                curNavType = slam.initPath()

            elif curControl == ControlType.DUMPING:
                time.sleep(DUMP_TIME)
                digDumpFlag = ControlType.DIGGING
                curNavType = slam.initPath()

            if control != curControl:
                curControl = control
                print(curControl)
        except Exception as e:
            print(e)
            running = False
