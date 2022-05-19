import tensorflow as tf
import time

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils

import rospy
from sensor_msgs.msg import Image as rosImage
from cv_bridge import CvBridge
import cv2


import numpy as np
import warnings
warnings.filterwarnings('ignore')  


data = None
conList = []
running = True

temp = True



PATH_TO_SAVED_MODEL = "./MRO/export/saved_model"

print('Loading model...', end='')
start_time = time.time()


detect_fn = tf.saved_model.load(PATH_TO_SAVED_MODEL)

end_time = time.time()
elapsed_time = end_time - start_time
print('Done! Took {} seconds'.format(elapsed_time))

def callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    # global temp
    # if temp:
    #     temp = False
    #     cv2.imwrite('test.png', cv_image)

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
                min_score_thresh=.7,
                skip_labels=True,
                skip_scores=False,
                agnostic_mode=False)
    
    min_score_thresho = 0.70
    
    # print([category_index.get(i) for i in detections['detection_classes'] if detections['detection_scores'][i] > min_score_thresho])
    #print(max(detections['detection_scores']))
    #out  = ['OH YEAH!!' for i in detections['detection_classes'] if detections['detection_scores'][0] > min_score_thresho]
    if max(detections['detection_scores']) > min_score_thresho:
        print('HELL YEh')
    # if len(out) > 0:
    #     print(out[0])
    else:
        print('nothing detected')
    
    cv2.imshow('rock detection',  cv2.resize(image_np_with_detections, (640, 480)))
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        global running
        running = False
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
    rospy.spin()
    # while running: 
    #     usrIn = input()
    #     if usrIn == 'q':
    #         running = False