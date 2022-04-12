#!/usr/bin/env python
import rospy
from rtabmap_ros.msg import MapGraph
import math
import numpy as np
from collections import deque
from controls import ControlType
from visualization import roversim

posResult = []
oriResult = []

RAD_TO_DEGREES = 180 / math.pi
ANGLE_THRESHOLD = 10 # degrees
DISTANCE_THRESHOLD = .1 # meters
ONTOP_THRESHOLD = .2 # meters

class Coord:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.rad = angle

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

        roversim.draw()
        roversim.ROVER.setPosition(roversim.Vector2(currentPos.y*100, currentPos.x*100))
        roversim.ROVER.setRotation(currentPos.rad + math.pi/2)
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
            self.followPath(currentPos)
            print(self.control)
            

    def followPath(self, currentPos:Coord) -> None:
        if len(self.path) > 0:
            targetPos = self.path[-1]
            targetAngle = math.atan2(targetPos.y-currentPos.y, targetPos.x-currentPos.x)

            cAng = currentPos.rad

            eucDist = math.sqrt((currentPos.x-targetPos.x)**2+(currentPos.y-targetPos.y)**2)
            if eucDist < ONTOP_THRESHOLD:
                targetPos = self.path.pop()
                self.control = ControlType.STOP
            elif (math.radians(10) < abs(targetAngle - cAng)):
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


    def wrap(angle):
        if angle > math.pi:
            angle = angle - 2*math.pi
        elif angle < -1*math.pi:
            angle = 2*math.pi + angle
        return angle

    def vizualize(self, currentPos):
        roversim.ROVER.setPosition(roversim.Vector2(currentPos.y*100, currentPos.x*100))
        pass

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

