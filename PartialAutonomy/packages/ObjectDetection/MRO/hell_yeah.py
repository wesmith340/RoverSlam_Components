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

SENDER_TYPE = Precedence.OBJECT_DETECTION
DANGER_DISTANCE = 1 # meters
AVOID_DISTANCE = .75 # meters
turningFlag = False
obstacle = None

object_width = 11
distance_object = 18

curDepthImage = None

class Obstacle:
    def __init__(self, dist, rad) -> None:
        self.dist = dist
        self.cumDist = dist
        self.numDetects = 1
        self.startRad = rad
    def addDetect(self, dist):
        self.cumDist += dist
        self.numDetects += 1
        self.dist = self.cumDist / self.numDetects

class TurnFlag:
    def __init__(self, flag:boolean=False) -> None:
        self.flag = flag



def depthCallback(data):
    global curDepthImage
    bridge = CvBridge()
    temp = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    # print('here')
    curDepthImage = np.array(temp)
    # pd.DataFrame(curDepthImage).to_csv('test.csv')
    # print(curDepthImage)
    # with open('test.csv', 'w') as w:
    #     w.write()
    #     # print(curDepthImage.shape)

turnFlag = TurnFlag()

def callback(data):
    global turnFlag
    if turnFlag.flag is False:
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
                    min_score_thresh=.85,
                    skip_labels=True,
                    skip_scores=False,
                    agnostic_mode=False)
        
        min_score_thresho = 0.90

        boxes = detections['detection_boxes']

        max_boxes = boxes.shape[0]

        scores = detections['detection_scores']
        # Y MIN XMIN YMAX X MAX
        global curDepthImage, obstacle, turningFlag
        detectFlag = False
        for i in range(min(max_boxes, boxes.shape[0])):
            if scores is None or scores[i] > min_score_thresho:

                dist = np.nanmean(curDepthImage[int(boxes[i][1]*640):int(boxes[i][3]*640),int(boxes[i][0]*480):int(boxes[i][2]*480)])
                print(dist)
                if dist < DANGER_DISTANCE:
                    # if obstacle is None:
                    #     obstacle = Obstacle(dist, slam.curPos.rad)
                    # else:
                    #     obstacle.addDetect(dist)
                    # detectFlag = True
                    # controller.sendCommand(SENDER_TYPE, ControlType.LEFT)
                    # print('left')
                    targetAngle = math.asin(AVOID_DISTANCE/dist)
                    targetDist = obstacle.dist/math.cos(targetAngle)
                    slam.avoidRock(targetAngle, targetDist, turningFlag)


                print(f'Dist to rock {i}: {dist}')
        # if detectFlag is False and obstacle is not None:
        #     controller.sendCommand(SENDER_TYPE, ControlType.NOTHING)
        #     print('Go!')
        #     slam.addPoint(obstacle.dist / math.cos(slam.curPos.rad - obstacle.startRad))
        #     obstacle = None


def listener():
    # rospy.init_node('obj_detection')
    rospy.Subscriber("/camera/rgb/image_rect_color", rosImage, callback)
    rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect", rosImage, depthCallback)


category_index = label_map_util.create_category_index_from_labelmap('./packages/ObjectDetection/MRO/MRO/label_map.pbtxt', use_display_name=True)
listener()

control = ControlType.NOTHING

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
