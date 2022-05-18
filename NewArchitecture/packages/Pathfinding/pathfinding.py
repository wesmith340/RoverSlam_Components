
from collections import deque
import math

from ..Serial_Commands import controller
from ..Serial_Commands.controls import ControlType, Precedence

from threading import Condition

import rospy
from rtabmap_ros.msg import MapGraph

class Coord:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.rad = angle

    def __str__(self):
        return '{'+ f'"x":{self.x},"y":{self.y}, "angle:"{self.degrees}'+'}'

class Slam:
    SENDER_TYPE = Precedence.PATH_FINDING
    ANGLE_THRESHOLD = 10 # degrees
    DISTANCE_THRESHOLD = .1 # meters
    ONTOP_THRESHOLD = .2 # meters
    DISTANCE_TO_DIGSITE = 5 # meters
    
    def __init__(self) -> None:
        self.path = deque()
        self.recordPath = deque()
        self.followFlag = False
        self.targetAngle = None
        self.rotatingFlag = False
        self.flag180 = False
        self.objFlag = None
        self.control = ControlType.NOTHING
        self.curPos = None
        self.condition = None

    def saveHome(self) -> None:
        self.home = self.curPos
        rad = self.curPos.rad
        x = self.curPos.x + Slam.DISTANCE_TO_DIGSITE * math.cos(rad)
        y = self.curPos.y + Slam.DISTANCE_TO_DIGSITE * math.sin(rad)
        self.path.clear()
        self.recordPath.clear()
        self.path.append(Coord(x, y, 0))

    def flipPath(self, c:Condition):
        self.condition = c
        temp = self.path
        temp.clear()
        self.path = self.recordPath
        self.recordPath = temp

    def startFollowing(self, c: Condition):
        self.condition = c
        self.followFlag = True
    def addPoint(self, dist, rad=None):
        if rad is None:
            rad = self.curPos.rad
        x = self.curPos.x + dist * math.cos(rad)
        y = self.curPos.y + dist * math.sin(rad)
        self.path.append(Coord(x, y, 0))

    def rotate180(self, c: Condition):
        self.condition = c
        self.flag180 = True
        if self.curPos.rad > 0:
            self.targetAngle = self.curPos.rad - math.pi
        else:
            self.targetAngle = self.curPos.rad + math.pi
        print(self.curPos.rad, self.targetAngle)
        self.followFlag = False
        self.rotatingFlag = True

    def avoidRock(self, turnAngle, targetDistance, turnFlag):
        self.objFlag = turnFlag
        self.objFlag.flag = True
        self.targetAngle = self.wrap(turnAngle+self.curPos.rad)

        self.addPoint(targetDistance, rad=self.targetAngle)
        self.followFlag = False
        self.turningFlag = True


    def rotate(self, currentPos:Coord):
        diff = abs(self.targetAngle - currentPos.rad)
        print(diff)
        if diff > math.pi:
            diff = 2*math.pi - diff
        if diff < math.radians(self.ANGLE_THRESHOLD):

            
            print(Precedence.NAVIGATION, ControlType.STOP)
            controller.sendCommand(Precedence.NAVIGATION, ControlType.STOP)
            controller.sendCommand(Precedence.NAVIGATION, ControlType.NOTHING)
            if self.flag180:
                self.condition.notify_all()
            else:
                self.objFlag.flag = False     

            self.followFlag = True
            self.rotatingFlag = False       
        else:
            print(Precedence.NAVIGATION, ControlType.RIGHT)
            controller.sendCommand(Precedence.NAVIGATION, ControlType.RIGHT)

    def callback(self, data):
        x = data.poses[-1].position.x
        y = data.poses[-1].position.y

        qZ = data.poses[-1].orientation.z
        w = data.poses[-1].orientation.w
        
        # Note still need to normalize quaternion before calculations
        angTheta = math.acos(w)*2
        if qZ < 0:
            angTheta *= -1
        # global currentPos
        self.curPos = Coord(x,y,angTheta)

        # print(self.followFlag, self.rotatingFlag)
        if (self.followFlag):
            if len(self.recordPath) == 0:
                self.recordPath.append(self.curPos)

            dist = math.sqrt((self.curPos.x-self.recordPath[-1].x)**2+(self.curPos.y-self.recordPath[-1].y)**2)
            if dist > Slam.DISTANCE_THRESHOLD:
                self.recordPath.append(self.curPos)

            self.followPath(self.curPos)
            print(self.control)
        elif (self.rotatingFlag):
            self.rotate(self.curPos)
                

    def followPath(self, currentPos:Coord) -> None:

        if len(self.path) > 0:
            targetPos = self.path[-1]
            targetAngle = math.atan2(targetPos.y-currentPos.y, targetPos.x-currentPos.x)

            cAng = currentPos.rad

            eucDist = math.sqrt((currentPos.x-targetPos.x)**2+(currentPos.y-targetPos.y)**2)
            if eucDist < self.ONTOP_THRESHOLD:
                targetPos = self.path.pop()
                self.control = ControlType.STOP

            elif (math.radians(self.ANGLE_THRESHOLD) < abs(targetAngle - cAng) 
                    and math.radians(self.ANGLE_THRESHOLD) < abs(math.pi-targetAngle + math.pi+cAng) 
                    and math.radians(self.ANGLE_THRESHOLD) < abs(math.pi+targetAngle + math.pi-cAng)):
                if targetAngle > cAng and targetAngle - cAng < math.pi or targetAngle < cAng and cAng - targetAngle > math.pi:
                    self.control = ControlType.LEFT

                else:
                    self.control = ControlType.RIGHT
            
            else:
                self.control = ControlType.FORWARD
        else:
            self.followFlag = False
            self.control = ControlType.NOTHING
            self.condition.notify_all()
        print(Slam.SENDER_TYPE, self.control)
        controller.sendCommand(Slam.SENDER_TYPE, self.control)


    def wrap(angle):
        if angle > math.pi:
            angle = angle - 2*math.pi
        elif angle < -1*math.pi:
            angle = 2*math.pi + angle
        return angle

    def listener(self):
        rospy.init_node('ekf_slam')
        rospy.Subscriber("/rtabmap/mapGraph", MapGraph, self.callback)

    def clear(self):
        self.path.clear()
        self.recordPath.clear()
        self.followFlag = False
        self.rotatingFlag = False

    def shutDown(self):
        rospy.destroy
slam = Slam()
slam.listener()
if __name__ == '__main__':
    print('Commands')
    print("  'r' : start recording")
    print("  'r' : stop recording")
    print("  's' : start pathfinding")
    print("  'q' : exit program")

    usrIn = input()

    while usrIn != 'q':
        if usrIn == 'r':
            slam.flipPath()
        elif usrIn == 's':
            print('Following path')
            slam.followFlag = not slam.followFlag
        usrIn = input()
