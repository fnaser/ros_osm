#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from txt_handler import save_data_to_file

# assumption: Hz(gps) < Hz(pose)
# assumption: pose and gps are in the same map_frame

global pose_array
global gps_array
global gps_array_viz
global latest_pose

def callbackGPS(data):
    global pose_array
    global gps_array
    global gps_array_viz

    pose_array.append(latest_pose[0])
    pose_array.append(latest_pose[1])
    gps_array.append(data.latitude)
    gps_array.append(data.longitude)
    gps_array_viz.append((data.latitude, data.longitude))

    rospy.loginfo(rospy.get_caller_id() + " I heard lat %f lon %f", data.latitude, data.longitude)

def callbackPose(data):
    global latest_pose

    latest_pose = []
    latest_pose.append(data.pose.pose.position.x)
    latest_pose.append(data.pose.pose.position.y)

    rospy.loginfo(rospy.get_caller_id() + " I heard x %f y %f", data.pose.pose.position.x, data.pose.pose.position.y)
    
def collect_data():
    global pose_array
    global gps_array
    global gps_array_viz
    global latest_pose
    latest_pose = []
    gps_array = []
    pose_array = []
    gps_array_viz = []

    rospy.init_node('collect_data', anonymous=True)
    fname_gps = rospy.get_param('~fname_gps', str("gps_data.txt"))
    fname_gps_viz = rospy.get_param('~fname_gps_viz', str("gps_viz_data.txt"))
    fname_pose = rospy.get_param('~fname_pose', str("pose_data.txt"))

    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callbackPose)
    rospy.Subscriber("ublox_gps/fix", NavSatFix, callbackGPS)

    rospy.spin()

    save_data_to_file(fname_gps, gps_array, '%.25e', ' ')
    save_data_to_file(fname_pose, pose_array, '%.25e', ' ')
    save_data_to_file(fname_gps_viz, gps_array_viz, '%.25f', ',')

if __name__ == '__main__':
    collect_data()

