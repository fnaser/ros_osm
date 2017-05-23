#!/usr/bin/env python
import rospy
import numpy as np

def save_data_to_file(fname, X, fmt_str, del_str):
    print fname
    np.savetxt(fname, X, fmt=fmt_str, delimiter=del_str, newline='\n', header='', footer='', comments='#')

def load_data_from_file(fname):
    print fname
    array = np.loadtxt(fname, dtype='float', comments='#', delimiter=' ', converters=None, skiprows=0, usecols=None, unpack=False, ndmin=0)
    print array
    return array
