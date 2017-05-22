#!/usr/bin/env python

import sys
import rospy
from custom_map_2_gps.srv import *
import numpy as np

def load_data_from_file(fname):
    print fname
    array = np.loadtxt(fname, dtype='float', comments='#', delimiter=' ', converters=None, skiprows=0, usecols=None, unpack=False, ndmin=0)
    print array
    return array

def save_data_to_file(fname, X):
    print fname
    np.savetxt(fname, X, fmt='%.25f', delimiter=',', newline='\n', header='', footer='', comments='#')

def transformer_client(map_x, map_y):
    rospy.wait_for_service('transfrom_map_to_gps')
    try:
        transformer_service = rospy.ServiceProxy('transfrom_map_to_gps', TransformMapToGps)
        resp = transformer_service(map_x, map_y)

        #print (resp.latitude, resp.longitude)
        #return resp.latitude, resp.longitude, resp.finished
        return resp.latitude, resp.longitude

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":

    fname_test_res_data = rospy.get_param('~fname_test_res_data', str("/home/fnaser/catvehicle_ws/src/ros_osm/data/test_results.txt"))
    fname_test_data = rospy.get_param('~fname_test_data', str("/home/fnaser/catvehicle_ws/src/ros_osm/data/pose_data.txt"))
    test_data = load_data_from_file(fname_test_data)

    #http://www.gpsvisualizer.com/

    test_data_res = []

    test_data_i = 0
    for i in range(len(test_data)):
        res = transformer_client(test_data[test_data_i], test_data[test_data_i+1])
        test_data_res.append((res[0],res[1]))

        print res

        if test_data_i + 3 < len(test_data):
           test_data_i = test_data_i + 2

    save_data_to_file(fname_test_res_data, test_data_res)

