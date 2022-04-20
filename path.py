from controls import ControlType
from collections import deque
from visualization import roversim
import math

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
    ANGLE_THRESHOLD = 10 # degrees
    DISTANCE_THRESHOLD = .1 # meters
    ONTOP_THRESHOLD = .2 # meters
    
    
    def __init__(self) -> None:
        self.path = deque()
        self.recordPath = deque()
        self.recordFlag = False
        self.followFlag = False
        self.control = ControlType.NOTHING

    def flipPath(self):
        temp = self.path
        temp.clear()
        self.path = self.recordPath
        self.recordPath = temp

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

        if len(self.recordPath) == 0:
            self.path.append(currentPos)
            roversim.ROVER.addToPath(roversim.Vector2(currentPos.x, currentPos.y))

        dist = math.sqrt((currentPos.x-self.path[-1].x)**2+(currentPos.y-self.path[-1].y)**2)
        if dist > Slam.DISTANCE_THRESHOLD:
            print('point taken')
            self.recordPath.append(currentPos)
            roversim.ROVER.addToPath(roversim.Vector2(currentPos.x, currentPos.y))
        
        if (self.followFlag):
            self.followPath(currentPos)
            # print(self.control)
                

    def followPath(self, currentPos:Coord) -> None:
        if len(self.path) > 0:
            targetPos = self.path[-1]
            targetAngle = math.atan2(targetPos.y-currentPos.y, targetPos.x-currentPos.x)

            cAng = currentPos.rad

            eucDist = math.sqrt((currentPos.x-targetPos.x)**2+(currentPos.y-targetPos.y)**2)
            if eucDist < Slam.ONTOP_THRESHOLD:
                targetPos = self.path.pop()
                self.control = ControlType.STOP
            elif (math.radians(Slam.ANGLE_THRESHOLD) < abs(targetAngle - cAng)):
                if targetAngle > cAng and targetAngle - cAng < math.pi or targetAngle < cAng and cAng - targetAngle > math.pi:
                    # print('TURN LEFT', f'{targetAngle:.2f} {cAng:.2f}')
                    self.control = ControlType.LEFT

                else:
                    # print('TURN RIGHT', f'{targetAngle:.2f} {cAng:.2f}')
                    self.control = ControlType.RIGHT
            else:
                # print('FORWARD TO:', eucDist)
                self.control = ControlType.FORWARD
        else:
            # print('END OF PATH')
            self.followFlag = False
            self.control = ControlType.NOTHING

    def listener(self):
        rospy.init_node('ekf_slam')
        rospy.Subscriber("/rtabmap/mapGraph", MapGraph, self.callback)


if __name__ == '__main__':
    slam = Slam()
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