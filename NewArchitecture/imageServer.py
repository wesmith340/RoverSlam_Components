import threading
from mlsocket import MLSocket
import numpy as np

import rospy
from sensor_msgs.msg import Image as rosImage
from cv_bridge import CvBridge

data = None
conList = []
running = True

# HOST = '169.254.157.5'
HOST = '192.168.50.243'
# HOST = 'localhost'
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
while input() != 'q':
    pass
