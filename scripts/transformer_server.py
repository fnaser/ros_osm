#!/usr/bin/env python

#import tensorflow as tf
import rospy
import numpy as np
import matplotlib.pyplot as plt

from custom_map_2_gps.srv import *

global w
global x_train
global y_train


# laser map pose
x_train = np.asarray([[444.151254874, 265.652920229, 1, 0, 0, 0],
                         [0, 0, 0, 444.151254874, 265.652920229, 1],
                         [453.713001542, 353.61225102, 1, 0, 0, 0],
                         [0, 0, 0, 453.713001542, 353.61225102, 1],
                         [39.9057006836, 501.173736572, 1, 0, 0, 0],
                         [0, 0, 0, 39.9057006836, 501.173736572, 1],
                         [219.53805542, 708.448913574, 1, 0, 0, 0],
                         [0, 0, 0, 219.53805542, 708.448913574, 1],
                         [530.371704102, 451.306335449, 1, 0, 0, 0],
                         [0, 0, 0, 530.371704102, 451.306335449, 1],
                         [313.398620605, 79.0848236084, 1, 0, 0, 0],
                         [0, 0, 0, 313.398620605, 79.0848236084, 1]])


# gps
y_train = np.asarray([1.3011672121136006, 103.79144668579102,
                         1.301180619700564,  103.79084318876266,
                         1.297304, 103.790299,
                         1.298275, 103.788218,
                         1.301397, 103.789476,
                         1.300623, 103.793024])


def load_data_from_file(fname):
    print fname
    array = np.loadtxt(fname, dtype='float', comments='#', delimiter=' ', converters=None, skiprows=0, usecols=None, unpack=False, ndmin=0)
    print array
    return array


def save_data_to_file(fname, X):
    print fname
    np.savetxt(fname, X, fmt='%.25e', delimiter=' ', newline='\n', header='', footer='', comments='#')


def normal_equation():
    global x_train
    global y_train
    global w

    print "y %f" % len(y_train)
    print "x %f" % len(x_train[:,0])
    print "x %f" % len(x_train[0,:])

    ne = np.matmul(x_train.T,x_train)
    w = np.matmul(np.linalg.inv(ne),np.matmul(x_train.T,y_train))

    print (x_train)
    print (y_train)


def transform_ne(coordinates):
    global w

    tmp_x = np.asarray([coordinates.map_x, coordinates.map_y, 1, 0, 0, 0])
    tmp_y = np.asarray([0, 0, 0, coordinates.map_x, coordinates.map_y, 1])

    return TransformMapToGpsResponse(tmp_x.dot(w), tmp_y.dot(w), True)


def construct_x_train_from_pose(pose_array):
    n = 6
    m = len(pose_array)

    tmp_pose_array = np.zeros((m,n))
    pose_array_i = 0
    for i in range(0,m):
        if i % 2 == 0:
           tmp_pose_array[i,0] = pose_array[pose_array_i]
           tmp_pose_array[i,1] = pose_array[pose_array_i+1]
           tmp_pose_array[i,2] = 1
        else:
           tmp_pose_array[i,3] = pose_array[pose_array_i]
           tmp_pose_array[i,4] = pose_array[pose_array_i+1]
           tmp_pose_array[i,5] = 1

           if pose_array_i + 3 < len(pose_array):
              pose_array_i = pose_array_i + 2

    pose_array = tmp_pose_array

    print pose_array

    return pose_array


def transformer_server():
    global x_train
    global y_train
    global w

    rospy.init_node('transformer_server')
    print ("### Start to transform from map to gps frame.")

    read_w_from_file = rospy.get_param('~read_w', bool(False))
    fname_w = rospy.get_param('~fname_w', str("/home/fnaser/w_data.txt"))
    fname_gps = rospy.get_param('~fname_gps', str("/home/fnaser/gps_data.txt"))
    fname_pose = rospy.get_param('~fname_pose', str("/home/fnaser/pose_data.txt"))

    if read_w_from_file:
       w = load_data_from_file(fname_w)
    else:
       print "### Start neq."
       tmp_pose_train = load_data_from_file(fname_pose)
       x_train = np.asarray(construct_x_train_from_pose(tmp_pose_train))
       y_train = load_data_from_file(fname_gps)
       normal_equation()
       save_data_to_file(fname_w, w)
    print w

    rospy.Service('transfrom_map_to_gps', TransformMapToGps, transform_ne)
    print ("### Ready to transform from map to gps frame.")
    rospy.spin()

if __name__ == "__main__":
    transformer_server()
