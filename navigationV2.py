#!/usr/bin/env python
import rospy
from rtabmap_ros.msg import MapGraph
import math
import numpy as np
from collections import deque
from controls import ControlType
import roversim

posResult = []
oriResult = []

RAD_TO_DEGREES = 180 / math.pi
ANGLE_THRESHOLD = 10 # degrees
DISTANCE_THRESHOLD = .5 # meters
ONTOP_THRESHOLD = .1 # meters

class Coord:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.rads = angle

    def __str__(self):
        return '{'+ \
            f'"x":{self.x},"y":{self.y}, "angle:"{self.degrees}'+'}'


class Slam:

    def __init__(self) -> None:
        self.path = deque()
        self.recordFlag = False
        self.followFlag = False
        self.control = ControlType.NOTHING

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
        currentPos = Coord(x,y,angTheta)

        if (self.recordFlag):
            if len(self.path) == 0:
                self.path.append(currentPos)
                roversim.ROVER.addToPath(roversim.Vector2(currentPos.x, currentPos.y))

            dist = math.sqrt((currentPos.x-self.path[-1].x)**2+(currentPos.y-self.path[-1].y)**2)
            if dist > DISTANCE_THRESHOLD:
                print('point taken')
                self.path.append(currentPos)
                roversim.ROVER.addToPath(roversim.Vector2(currentPos.x, currentPos.y))
        elif (self.followFlag):
            print(self.followPath(currentPos))


    def pointToPoint(self, currentPos:Coord) -> ControlType:
        if len(self.path) > 0:
            targetPos = self.path[-1]
            targetAngle = math.atan2(targetPos.y-currentPos.y, targetPos.x-currentPos.x)

            cAng = currentPos.rad

            eucDist = math.sqrt((currentPos.x-targetPos.x)**2+(currentPos.y-targetPos.y)**2)
            if eucDist < ONTOP_THRESHOLD:
                targetPos = self.path.pop()
                control = ControlType.STOP
                # for point in path:
                #     print(point.strPos())
                #     print()
            elif (math.radians(10) < abs(targetAngle - cAng)):
                if targetAngle > cAng and targetAngle - cAng < math.pi or targetAngle < cAng and cAng - targetAngle > math.pi:
                    # print('TURN LEFT', f'{targetAngle:.2f} {cAng:.2f}')
                    self.control = ControlType.LEFT

                    # angle += math.radians(5)
                    # angle = wrap(angle)
                else:
                    # print('TURN RIGHT', f'{targetAngle:.2f} {cAng:.2f}')
                    self.control = ControlType.RIGHT
                    # angle -= math.radians(5)
                    # angle = wrap(angle)

            
            else:
                # x = currentPos.x + 5*math.cos(angle)
                # y = currentPos.y + 5*math.sin(angle)
                # currentPos = roversim.Vector2(x,y)
                # print('FORWARD TO:', eucDist)
                self.control = ControlType.FORWARD
        else:
            print('END OF PATH')
            self.control = ControlType.NOTHING


    def wrap(angle):
        if angle > math.pi:
            angle = angle - 2*math.pi
            pass
        elif angle < -1*math.pi:
            angle = 2*math.pi + angle
            pass
        return angle

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
    rospy.Subscriber("/rtabmap/mapGraph", MapGraph, slam.callback)
    # 

slam = Slam()

if __name__ == '__main__':
    listener()

    print('Commands')
    print("  'r' : start recording")
    print("  'r' : stop recording")
    print("  's' : start pathfinding")
    print("  'q' : exit program")

    usrIn = input()

    while usrIn != 'q':
        if slam.recordFlag == False and usrIn == 'r':
            print('Starting path recording')
            slam.recordFlag = True
            slam.path.clear()
        elif slam.recordFlag and usrIn == 'r':
            print('Stopping path recording')
            printPath(slam.path)
            slam.recordFlag = False
        elif usrIn == 's':
            if len(slam.path) > 0:
                print('Following path')
                slam.followFlag = True
            else:
                print('Error: No Path to follow')
        usrIn = input()

