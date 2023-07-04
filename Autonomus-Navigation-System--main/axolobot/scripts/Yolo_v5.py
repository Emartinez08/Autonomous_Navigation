#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np

class YOLOv5DetectorNode:
    def __init__(self):
        rospy.init_node('yolov5_detector_node')
        self.model = torch.hub.load('ultralytics/yolov5', 'custom',
                                    path='/home/kike/catkin_ws/src/beginner_tutorials/scripts/elbuenomasbueno.pt')

        # Create publishers for detections and area
        self.detection_pub_1 = rospy.Publisher('/traffic/sign_1', String, queue_size=1)
        self.detection_pub_2 = rospy.Publisher('/traffic/sign_2', String, queue_size=1)
        self.area_pub_1 = rospy.Publisher('/traffic/sign_area_1', Float32, queue_size=1)
        self.area_pub_2 = rospy.Publisher('/traffic/sign_area_2', Float32, queue_size=1)

        # Subscribe to the camera topic
        rospy.Subscriber("/video_source/raw", Image, self.camera_callback)

        # Create a CvBridge object
        self.bridge = CvBridge()

    def camera_callback(self, msg):
        # Convert the ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Perform detections
        detect = self.model(frame)

        info = detect.pandas().xyxy[0]  # Predictions from the image
        
        if info.empty:
            detected_class = 'Nothing'
            box_area = 0.0
            detected_class_2 = 'Nothing'
            box_area_2 = 0.0

        elif len(info['name']) == 1:
            detected_class = info.at[0,'name']
            detected_class_2 = 'Nothing'
            box = info.at[0, 'xmin'], info.at[0, 'ymin'], info.at[0, 'xmax'], info.at[0, 'ymax']
            box_width = box[2] - box[0]
            box_height = box[3] - box[1]
            box_area = box_width * box_height
            box_area_2 = 0.0
            print("Detected Class:", detected_class)
            print("Box Width:", box_width)
            print("Box Height:", box_height)
            print("Box Area:", box_area)
            print("---")

        elif len(info['name']) > 1:
            detected_class = info.at[0,'name']
            box = info.at[0, 'xmin'], info.at[0, 'ymin'], info.at[0, 'xmax'], info.at[0, 'ymax']
            box_width = box[2] - box[0]
            box_height = box[3] - box[1]
            box_area = box_width * box_height
            print("Detected Class:", detected_class)
            print("Box Width:", box_width)
            print("Box Height:", box_height)
            print("Box Area:", box_area)
            print("---")
            detected_class_2 = info.at[1,'name']
            box_2 = info.at[1, 'xmin'], info.at[1, 'ymin'], info.at[1, 'xmax'], info.at[1, 'ymax']
            box_width_2 = box_2[2] - box_2[0]
            box_height_2 = box_2[3] - box_2[1]
            box_area_2 = box_width_2 * box_height_2
            print("Detected Class_2:", detected_class_2)
            print("Box Width:", box_width_2)
            print("Box Height:", box_height_2)
            print("Box Area:", box_area_2)
            print("---")

        self.detection_pub_1.publish(detected_class)
        self.area_pub_1.publish(box_area)
        self.detection_pub_2.publish(detected_class_2)
        self.area_pub_2.publish(box_area_2)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector_node = YOLOv5DetectorNode()
        detector_node.run()
    except rospy.ROSInterruptException:
        pass