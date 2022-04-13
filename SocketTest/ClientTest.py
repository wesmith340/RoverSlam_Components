# Import socket module
from mlsocket import MLSocket            
import pickle
import cv2
 

# HOST = '169.254.27.83'
HOST = 'localhost'
PORT = 5001

IMG_SIZE = 921765
with MLSocket() as s:
    s.connect((HOST, PORT)) # Connect to the port and host

    while True:
        img = s.recv(IMG_SIZE)
        # img = pickle.loads(data)
        cv2.imshow('test2',  cv2.resize(img, (800, 600)))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
