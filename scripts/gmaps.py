#!/usr/bin/env python

import rospy
import googlemaps

def gmaps_nearest_roads(latitude, longitude):

    gmaps = googlemaps.Client(key='AIzaSyBfOXk8wbCSvUHBZjnwFupIK42cQi8DXts')

    nearest_roads_result = gmaps.nearest_roads((latitude, longitude))
    if len(nearest_roads_result) > 0:
       print nearest_roads_result[0]
       latitude = nearest_roads_result[0]['location']['latitude']
       longitude = nearest_roads_result[0]['location']['longitude']

    print "Result of gmaps.nearest_roads:"
    print (latitude, longitude)

    return latitude, longitude

