# Install

 - Ubuntu 16.04
 - ROS Kinetic
 - catkin_make
 - Overpass http://overpass-api.de/no_frills.html#startup

# How to use ros_osm

1) Collect data
2) Calculate transformation
3) Run queries

![alt text](https://github.com/fnaser/ros_osm/blob/master/tmatrix.PNG)

```
roscore
rosbag play <filename>.bag
roslaunch ros_osm collect_data.launch
[Ctrl+C]
roslaunch ros_osm neq.launch
# settings in launch file:
# read_w = false
[Ctrl+C]
# change settings to read_w = true
roslaunch ros_osm ros_osm.launch
```
# Visualize data

 - http://www.gpsvisualizer.com/map_input?form=data
 - Copy&Paste data e.g. from 'test_results.txt'
