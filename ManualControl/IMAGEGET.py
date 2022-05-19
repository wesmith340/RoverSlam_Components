# Import socket module
from mlsocket import MLSocket
import pickle
import cv2
 

# HOST = '169.254.27.83'
HOST = '192.168.50.243'
# HOST = 'localhost'
PORT = 5001
# IMG_SIZE = 921765
MSG_LEN = 4096

with MLSocket() as s:
    s.connect((HOST, PORT))
    while True:
        img = s.recv(MSG_LEN)
        cv2.imshow('test2',  cv2.resize(img, (800, 600)))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
