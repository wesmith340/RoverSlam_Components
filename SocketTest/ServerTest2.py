import cv2
import numpy as np
import pickle
from mlsocket import MLSocket
import threading

HOST = '169.254.157.5'
# HOST = 'localhost'
PORT = 5001

data = None
conList = []
running = True

IMG_SIZE = 921765
img = cv2.imread('test.jpeg')
cv2.imshow('window', img)
img = np.array(img)
def acceptCon():
    global conList, img
    with MLSocket() as s:
        while True:
            try:
                s.bind((HOST, PORT))
                s.listen()
                conn, address = s.accept()
                # conList.append(conn)
                conn.send(img)
                print('connection from', address)
            except:
                break

t = threading.Thread(target=acceptCon)
t.start()

# define a video capture object
# vid = cv2.VideoCapture(0)

while(running):
    # Capture the video frame
    # ret, frame = vid.read()
    # data = np.array(frame)
    # print(len(data))
    # for c in conList:
    #     try:
    #         c.send(data)
    #     except Exception as e:
    #         print(e)
    #         conList.remove(c)
    #         c.close()
    cv2.imshow('window', img)
    if cv2.waitKey(1) & 0xFF == 27:
        break 

 

