#!/usr/bin/env python 

import rospy
import numpy as np
import time
from place_to_box.srv import data, dataRequest, dataResponse
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
import cv2

def placing():
    print("hello")
    rospy.wait_for_service('place')

    try :
        place_client = rospy.ServiceProxy('place', data)
        command = dataRequest()
        command.pose.position.x = 0.45
        command.pose.position.y = 0.0
        command.pose.position.z = 0.05
        command.pose.orientation.x = 0.0
        command.pose.orientation.y = 0.707
        command.pose.orientation.z = 0.0
        command.pose.orientation.w = 0.707
        command.id = 0
        res = place_client(command)
        
    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
	
    placing()
    print("place client")
