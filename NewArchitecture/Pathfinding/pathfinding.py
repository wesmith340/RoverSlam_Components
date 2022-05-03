from controls import ControlType
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
    
    
    def __init__(self) -> None:
        self.path = deque()
        self.recordPath = deque()
        self.recordFlag = False
        self.followFlag = False
        self.targetAngle = None
        self.rotatingFlag = False
        self.control = ControlType.NOTHING
        self.curPos = None
        self.condition = None

    def saveHome(self) -> None:
        self.home = self.curPos

    def flipPath(self):
        temp = self.path
        temp.clear()
        self.path = self.recordPath
        self.recordPath = temp

    def startFollowing(self, c: Condition):
        self.condition = c
        self.followFlag = True

    def rotate180(self, c: Condition):
        self.condition = c
        if self.curPos.rad > 0:
            self.targetAngle = self.curPos.rad - math.pi
        else:
            self.targetAngle = self.curPos.rad + math.pi
        self.followFlag = False
        self.rotatingFlag = True

    def rotate(self, currentPos:Coord):
        if (math.radians(self.ANGLE_THRESHOLD) < abs(self.targetAngle - currentPos.rad) 
                or math.radians(self.ANGLE_THRESHOLD) < abs(math.pi-self.targetAngle + math.pi+currentPos.rad) 
                or math.radians(self.ANGLE_THRESHOLD) < abs(math.pi+self.targetAngle + math.pi-currentPos.rad)):

            controller.sendCommand(Precedence.NAVIGATION, ControlType.STOP)
            controller.sendCommand(Precedence.NAVIGATION, ControlType.NOTHING)
            self.condition.notify_all()
        else:    
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

        if (self.recordFlag):
            if len(self.path) == 0:
                self.path.append(self.curPos)

            dist = math.sqrt((self.curPos.x-self.path[-1].x)**2+(self.curPos.y-self.path[-1].y)**2)
            if dist > Slam.DISTANCE_THRESHOLD:
                print('point taken')
                self.path.append(self.curPos)

        elif (self.followFlag):
            self.followPath(self.curPos)
            print(self.control)
                

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
                    or math.radians(self.ANGLE_THRESHOLD) < abs(math.pi-targetAngle + math.pi+cAng) 
                    or math.radians(self.ANGLE_THRESHOLD) < abs(math.pi+targetAngle + math.pi-cAng)):
                if targetAngle > cAng and targetAngle - cAng < math.pi or targetAngle < cAng and cAng - targetAngle > math.pi:
                    print('TURN LEFT', f'{targetAngle:.2f} {cAng:.2f}')
                    self.control = ControlType.LEFT

                else:
                    print('TURN RIGHT', f'{targetAngle:.2f} {cAng:.2f}')
                    self.control = ControlType.RIGHT
            
            else:
                print('FORWARD TO:', eucDist)
                self.control = ControlType.FORWARD
        else:
            print('END OF PATH')
            self.followFlag = False
            self.control = ControlType.NOTHING
            self.condition.notify_all()

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

slam = Slam()

if __name__ == '__main__':

    slam.listener()
    print('Commands')
    print("  'r' : start recording")
    print("  'r' : stop recording")
    print("  's' : start pathfinding")
    print("  'q' : exit program")

    usrIn = input()

    while usrIn != 'q':
        if slam.recordFlag == False and usrIn == 'r':
            slam.flipPath()
        elif usrIn == 's':
            print('Following path')
            slam.followFlag = not slam.followFlag
        usrIn = input()