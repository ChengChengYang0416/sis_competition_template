#!/usr/bin/env python

import sys
import rospy
#import numpy as np
#import time
#from GoToPos.srv  import data, dataRequest, dataResponse
from astar.srv import GoToPos, GoToPosResponse, GoToPosRequest


def navigation(x,y):
    print("hello")
    rospy.wait_for_service('to_position')

    try :
        navigation_client = rospy.ServiceProxy('to_position', GoToPos)
        command = GoToPosRequest()
        command.pos = x
        response = navigation_client(command)
        
        navigation_client = rospy.ServiceProxy('get_pose', GoToPos)
        command = GoToPosRequest()      
        command.plate = y        
        response = navigation_client(command)
        
    
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
	

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print("out")
        sys.exit(1)
    navigation(x, y)

    
