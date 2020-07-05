#!/usr/bin/env python 

import rospy
import numpy as np
import time
from place_to_box.srv import data, dataRequest, dataResponse
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
import cv2

def detect():
    originalImage = cv2.imread('test.png')
    grayImage = cv2.cvtColor(originalImage, cv2.CV_BGR2GRAY)
    (thresh, blackAndWhiteImage) = cv2.threshold(grayImage, 127, 255, cv2.THRESH_BINARY)
    cv2.imshow('Black white image', blackAndWhiteImage)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
	
    detect()
    print("detect")
