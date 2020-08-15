#!/usr/bin/env python

import rospy
import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Camera:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_received = False
        self.image = [[]]
        
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.ImageCallback)
        rospy.sleep(1)

    def ImageCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_received = True
            self.image = cv_image
        except CvBridgeError as e:
            rospy.logerr(e)

    def take_photo(self, img_title):
        if self.image_received:
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False

if __name__ == "__main__":
    rospy.init_node("vision", anonymous = False)
    camera = Camera()
    #cv2.nameWindow('video')
    #lower_color = np.array([0,0,0])
    #upper_color = np.array([0,0,0])

    while not rospy.is_shutdown():
        hsv = cv2.cvtColor(camera.image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0,120,70])
        upper_red = np.array([10,255,255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170,120,70])
        upper_red = np.array([180,255,255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2

        result = cv2.bitwise_and(hsv, hsv, mask = mask)
        result = cv2.cvtColor(result, cv2.COLOR_HSV2BGR)
        print(result)
        cv2.imshow('video', camera.image)
        cv2.imshow('filtered_video', result)
        cv2.waitKey(10) 