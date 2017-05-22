# Install

 - Ubuntu 16.04
 - ROS Kinetic
 - catkin_make

# How to use ros_osm

1) collect data
2) calculate transformation
3) run queries

```
roscore
cd data
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
