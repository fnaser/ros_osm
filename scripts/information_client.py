#!/usr/bin/env python

import rospy
from custom_map_2_gps.srv import *
import numpy as np

def information_client(map_x, map_y):
    rospy.wait_for_service('get_information_of_gps')
    try:
        information_service = rospy.ServiceProxy('get_information_of_gps', GetInformation)
        resp = information_service(map_x, map_y)
        print resp

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":

    #test_data = np.asarray([710.186677414, 109.308316028])
    #test_data = np.asarray([692.55917328147305056518235, 674.15354638938072184828343])
    test_data = np.asarray([196.76704727968336783305858, 628.25255767747557911206968])
    information_client(test_data[0], test_data[1])

