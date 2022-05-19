import serial, time
from serial.tools.list_ports import comports
from enum import Enum

ports = comports()
for c in ports:
    print(c)

def sendCommand(port, command):
    port.write(command)


arduino = serial.Serial(port=ports[0].device, baudrate=9600)

time.sleep(1) #give the connection a second to settle

class ControlType(Enum):
    FORWARD = 0
    RIGHT = 1
    LEFT = 2
    STOP = 3
    BACKWARD = 4
    EXTEND_DIG = 5
    RETRACT_DIG = 6
    START_DIG_DRIVE = 7
    DEPOSIT_UP = 8
    DEPOSIT_DOWN = 9
    NOTHING = -1

import threading
from mlsocket import MLSocket
import numpy as np

import rospy
from sensor_msgs.msg import Image as rosImage
from cv_bridge import CvBridge

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

# Variable to keep our game loop running
running = True
  

keys = {
    'w':ControlType.FORWARD,
    's':ControlType.BACKWARD,
    'a':ControlType.LEFT,
    'd':ControlType.RIGHT,
    '':ControlType.STOP,
    '1':ControlType.EXTEND_DIG,
    '2':ControlType.RETRACT_DIG,
    '3':ControlType.START_DIG_DRIVE,
    '4':ControlType.DEPOSIT_UP,
    '5':ControlType.DEPOSIT_DOWN

}

while running:
    print(keys)
    print('q to quit')
    usrIn = input()
    if usrIn in keys:
        arduino.write(f'{keys[usrIn].value}'.encode())
        print(keys[usrIn])
    elif usrIn == 'q':
        running = False
        break