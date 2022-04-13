# Import socket module
import socket 
import pickle
import cv2
 

# HOST = '169.254.27.83'
HOST = 'localhost'
PORT = 5001
IMG_SIZE = 921765
MSG_LEN = 4096

def recieve(s):
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
    return pickle.loads(arr)


with socket.socket() as s:
    s.connect((HOST, PORT)) # Connect to the port and host

    while True:
        img = recieve(s)
        # img = pickle.loads(data)
        cv2.imshow('test2',  cv2.resize(img, (800, 600)))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
