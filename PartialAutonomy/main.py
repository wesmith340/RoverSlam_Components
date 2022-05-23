import serial, time
from serial.tools.list_ports import comports
from enum import Enum

import threading
from mlsocket import MLSocket
import numpy as np

import rospy
from sensor_msgs.msg import Image as rosImage
from cv_bridge import CvBridge

from packages.Pathfinding.pathfinding import slam
from packages.Pathfinding.qrfinding import qrFinder
from packages.Serial_Commands import controller
from packages.Serial_Commands.controls import ControlType, Precedence

SENDER_TYPE = Precedence.NAVIGATION
DIG_TIME = 1000
DUMP_TIME = 1000

ports = comports()
for c in ports:
    print(c)


def sendCommand(port, command):
    port.write(command)


arduino = serial.Serial(port=ports[0].device, baudrate=9600)

time.sleep(1) #give the connection a second to settle

data = None
conList = []
running = True

HOST = '192.168.50.243'
PORT = 5001

temp = True

def acceptCon():
    global conList, running
    
    with MLSocket() as s:
        s.bind((HOST, PORT))
        s.listen()
        while running:
            try:
                conn, address = s.accept()
                conList.append(conn)
                print('connection from', address)  
            except:
                break
    

def callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    image_np = np.array(cv_image)

    for c in conList:
        try:
            c.sendall(image_np)
        except Exception as e:
            print(e)
            conList.remove(c)
            c.close()

def listener():
    rospy.init_node('image_server')
    rospy.Subscriber("/camera/rgb/image_rect_color", rosImage, callback)

t = threading.Thread(target=acceptCon)
listener()
t.start()

print('listening')

class Waiter(threading.Condition):
    def wait(self):
        self.acquire()
        super().wait()
        self.release()
    
    def notify_all(self) -> None:
        self.acquire()
        super().notify_all()
        self.release()

running = True
  

keys = {
    'w':ControlType.FORWARD,
    's':ControlType.BACKWARD,
    'a':ControlType.LEFT,
    'd':ControlType.RIGHT,
    '':ControlType.STOP,
    '1':ControlType.DIGGING,
    '2':ControlType.DUMPING,
}

def simpleNavLoop():
    global c, threadRunning
    slam.clear()

    if threadRunning:
        slam.saveHome()
        slam.startFollowing(c)
        print('Following path')
        c.wait()

    if threadRunning:
        controller.sendCommand(SENDER_TYPE, ControlType.DIGGING)
        print(SENDER_TYPE, ControlType.DIGGING)
        time.sleep(DIG_TIME)
        controller.sendCommand(SENDER_TYPE, ControlType.STOP)
        print(SENDER_TYPE, ControlType.STOP)

    if threadRunning:
        slam.flipPath(c)
        print('Following path')
        c.wait()

c = Waiter()
threadRunning = False
if __name__ == '__main__':
    running = True
    while running:

        inManual = True
        while inManual:
            usrIn = input()
            if usrIn in keys:
                controller.sendCommand(Precedence.MANUAL, keys[usrIn])
                print(keys[usrIn])
            elif usrIn == '3':
                controller.sendCommand(Precedence.MANUAL, ControlType.NOTHING)
                inManual = False
        
        threadRunning = True

        print('Starting partial autonomy')
        partAutoThread = threading.Thread(target=simpleNavLoop)
        partAutoThread.start()
        print('press enter to return to manual')
        print('q to quit')
        usrIn = input()

        threadRunning = False
        c.notify_all()

        if usrIn == 'q':
            running = False