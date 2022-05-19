from xmlrpc.client import boolean
import tensorflow as tf
import time
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils
import math
from packages.Pathfinding.pathfinding import slam
from packages.Serial_Commands import controller
from packages.Serial_Commands.controls import ControlType, Precedence

import rospy
from sensor_msgs.msg import Image as rosImage
from cv_bridge import CvBridge



import numpy as np
import warnings
warnings.filterwarnings('ignore')  

PATH_TO_SAVED_MODEL = "./packages/ObjectDetection/MRO/MRO/export/saved_model"

print('Loading model...', end='')
start_time = time.time()


detect_fn = tf.saved_model.load(PATH_TO_SAVED_MODEL)

end_time = time.time()
elapsed_time = end_time - start_time
print('Done! Took {} seconds'.format(elapsed_time))


class QRFinder:
    SENDER_TYPE = Precedence.QR_CODE
    MIN_CERTAINTY = .70
    CENTER_THRESHOLD = 20 # pixels
    MAX_BOXES = 1
    def __init__(self) -> None:
        self.lookingFlag = False
        self.curDepthImage = None
        self.lastDetectTime = 0
        pass

    def findQR(self, c):
        # TODO
        c.notify_all()

    
    def depthCallback(self, data):
        global curDepthImage
        bridge = CvBridge()
        temp = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        self.curDepthImage = np.array(temp)


    def callback(self, data):
        if self.lookingFlag:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            image_np = np.array(cv_image)

            input_tensor = tf.convert_to_tensor(np.expand_dims(image_np, 0), dtype=tf.uint8)
            input_tensor = input_tensor[:, :, :, :3]
            detections = detect_fn(input_tensor)
            
            num_detections = int(detections.pop('num_detections'))
            detections = {key: value[0, :num_detections].numpy()
                        for key, value in detections.items()}
            
            detections['num_detections'] = num_detections
            detections['detection_classes'] = detections['detection_classes'].astype(np.int64)
            boxes = detections['detection_boxes']

            scores = detections['detection_scores']

            # Y MIN XMIN YMAX X MAX

            for i in range(min(QRFinder.MAX_BOXES, boxes.shape[0])):
                if scores is None or scores[i] > QRFinder.MIN_CERTAINTY:
                    
                    self.lastDetectTime = time.time()

                    # Center image
                    boxCenter = math.round((boxes[i][1]+boxes[i][3])*640/2)

                    if boxCenter < 640-QRFinder.CENTER_THRESHOLD:
                        print('turn right')
                    elif boxCenter > 640-QRFinder.CENTER_THRESHOLD:
                        print('turn left')
                    else:
                        dist = np.nanmean(self.curDepthImage[int(boxes[i][1]*640):int(boxes[i][3]*640),int(boxes[i][0]*480):int(boxes[i][2]*480)])
                        print(f'Dist to QR code {i}: {dist}')
                        print('centered')

            if boxes.shape[0] == 0: # no detections
                curTime = time.time()
                if curTime - self.lastDetectTime < 1000: # Wait to see if QR is in frame
                    pass
                elif curTime - self.lastDetectTime < 10000: # Spin for QR code
                    pass
                else: # Random direction thing
                    pass

                    


qrFinder = QRFinder()


def listener():
    rospy.Subscriber("/camera/rgb/image_rect_color", rosImage, qrFinder.callback)
    rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect", rosImage, qrFinder.depthCallback)


category_index = label_map_util.create_category_index_from_labelmap('./packages/ObjectDetection/MRO/MRO/label_map.pbtxt', use_display_name=True)
listener()


if __name__ == '__main__':
    running = True
    rospy.spin()
