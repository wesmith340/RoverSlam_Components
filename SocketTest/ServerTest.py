from ctypes import sizeof
import cv2
from dataFrame import DataFrame, HOST, PORT
import pickle
import socket
import threading

data = None
conList = []
running = True

imageSize = 4096

def acceptCon():
    global running
    s = socket.socket()        
    print ("Socket successfully created")
    
    s.bind((HOST, PORT))        
    print ("socket binded to %s" %(PORT))
    
    s.listen(5)    
    while running:
        try:
            c, addr = s.accept()    
            print ('Got connection from', addr )
            c.send(imageSize.to_bytes(4, 'big'))
            conList.append(c)
        except Exception as e:
            print(e)
            for c in conList:
                c.close()
            running = False

t = threading.Thread(target=acceptCon)
t.start()

# define a video capture object
vid = cv2.VideoCapture(0)
while(running):
    # Capture the video frame
    ret, frame = vid.read()
    data = pickle.dumps(DataFrame(frame))
    imageSize = len(data)
    for c in conList:
        try:
            c.send(data)
        except Exception as e:
            print(e)
            conList.remove(c)
            c.close()

 

