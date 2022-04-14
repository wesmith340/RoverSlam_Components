# Import socket module
from mlsocket import MLSocket
import pickle
import socket
import cv2
 

# HOST = '169.254.27.83'
HOST = 'localhost'
PORT = 5001
IMG_SIZE = 921765
MSG_LEN = 4096
HEADER_SIZE = 10

def recieve(s):
    size = int(s.recv(HEADER_SIZE).decode('utf8'))
    pos = 0
    fragments = []
    chunkSize = MSG_LEN
    while chunkSize > 0:
        print(pos, size)
        frag = s.recv(chunkSize)
        pos += len(frag)
        fragments.append(frag)
        if size - pos > MSG_LEN:
            chunkSize = MSG_LEN
        else:
            chunkSize = size - pos

    print(pos, size)
    data = b''.join(fragments)
    img = pickle.loads(data)
    return img

with socket.socket() as s:
    s.connect((HOST, PORT))
    
    while True:
        img = recieve(s)
        cv2.imshow('test2',  cv2.resize(img, (800, 600)))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
