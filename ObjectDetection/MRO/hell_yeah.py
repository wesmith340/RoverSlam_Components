
import os
import pathlib
import tensorflow as tf
import time

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils
import cv2

import rospy
from sensor_msgs.msg import Image as rosImage
from cv_bridge import CvBridge

import threading
import socket
import pickle
from dataFrame import DataFrame, HOST, PORT

import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings('ignore')  


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



PATH_TO_SAVED_MODEL = "./MRO/export/saved_model"

print('Loading model...', end='')
start_time = time.time()


detect_fn = tf.saved_model.load(PATH_TO_SAVED_MODEL)

end_time = time.time()
elapsed_time = end_time - start_time
print('Done! Took {} seconds'.format(elapsed_time))

def callback(data):
    global imageSize

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    im = pickle.dumps(DataFrame(cv_image))
    imageSize = len(im)
    
    for c in conList:
        try:
            c.send(data)
        except Exception as e:
            print(e)
            conList.remove(c)
            c.close()



    image_np = np.array(cv_image)
    input_tensor = tf.convert_to_tensor(np.expand_dims(image_np, 0), dtype=tf.uint8)
    input_tensor = input_tensor[:, :, :, :3]
    detections = detect_fn(input_tensor)
    
    num_detections = int(detections.pop('num_detections'))
    detections = {key: value[0, :num_detections].numpy()
                  for key, value in detections.items()}
    detections['num_detections'] = num_detections


    detections['detection_classes'] = detections['detection_classes'].astype(np.int64)
    
    label_id_offset = 1
    image_np_with_detections = image_np.copy()

    viz_utils.visualize_boxes_and_labels_on_image_array(
                image_np_with_detections,
                detections['detection_boxes'],
                detections['detection_classes']+label_id_offset,
                detections['detection_scores'],
                category_index,
                use_normalized_coordinates=True,
                max_boxes_to_draw=1,
                min_score_thresh=.9,
                skip_labels=True,
                skip_scores=True,
                agnostic_mode=False)
    
    min_score_thresho = 0.7
    

    # print([category_index.get(i) for i in detections['detection_classes'] if detections['detection_scores'][i] > min_score_thresho])
    
    out  = ['OH YEAH!!' for i in detections['detection_classes'] if detections['detection_scores'][i] > min_score_thresho]
    
    if len(out) > 0:
        print(out[0])
    else:
        print('nothing detected')
    
    cv2.imshow('rock detection',  cv2.resize(image_np_with_detections, (800, 600)))
    
    if cv2.waitKey(10) & 0xFF == ord('q'):
        cv2.destroyAllWindows()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('obj_detection')
    rospy.Subscriber("/camera/rgb/image_rect_color", rosImage, callback)
    # 

category_index = label_map_util.create_category_index_from_labelmap('./MRO/label_map.pbtxt', use_display_name=True)
listener()

# cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
# width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
# height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
if __name__ == '__main__':
    running = True
    
    while running: 
        # if cv2.waitKey(10) & 0xFF == ord('q'):
        #     running = False
        #     cv2.destroyAllWindows()
        #     break
        usrIn = input()
        if usrIn == 'q':
            running = False
