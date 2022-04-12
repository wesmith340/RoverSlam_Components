# Import socket module
import socket            
import pickle
from dataFrame import DataFrame, HOST, PORT
import cv2
 
# Create a socket object
s = socket.socket()        

MSG_LEN = 4096
 
# connect to the server on local computer
s.connect((HOST,PORT))

IMG_SIZE = int.from_bytes(s.recv(4), 'big')

while True:
    fragments = []
    pos = 0
    while pos < IMG_SIZE:
        if pos + MSG_LEN > IMG_SIZE:
            packet = s.recv(IMG_SIZE-pos)
        else:
            packet = s.recv(MSG_LEN)
        
        
        if not packet:
            break
  
        fragments.append(packet)
        pos += len(packet)

    
    arr = b''.join(fragments)
    data = pickle.loads(arr)
    cv2.imshow('test2',  cv2.resize(data.image, (800, 600)))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
