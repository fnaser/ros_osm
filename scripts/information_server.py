#!/usr/bin/env python

#import tensorflow as tf
import rospy
import googlemaps
import xml.etree.ElementTree
import numpy as np
import overpy

from custom_map_2_gps.srv import *
from gmaps import gmaps_nearest_roads

global call_gmaps_nearest_roads
global apioverpy
global nodePointList
global nodeObjectList

#https://github.com/shashwat14/OsmPy

class Map(object):
    
    def __init__(self, map_file):
        self.root = xml.etree.ElementTree.parse(map_file).getroot()
        self.tags = set()
        children = self.root.getchildren()
        for child in children:
            self.tags.add(child.tag)
        boundElement = children[0]
        self.minlat = float(boundElement.get('minlat'))
        self.maxlat = float(boundElement.get('maxlat'))
        self.minlon = float(boundElement.get('minlon'))
        self.maxlon = float(boundElement.get('maxlon'))
        
    def getTags(self):
        '''
        Param: None
        Return: set of available tags for given map
        '''
        return self.tags
    
    def getElementByTag(self, tag, element=None):
        '''
        Param: tag as string. Options available - way, node, bounds, member
        Return: list of element with given tag
        '''
        if element is not None:
            return element.findall(tag)
        return self.root.findall(tag)
    
    def getNodeTags(self):
        '''
        Param: None
        Return: Set of available tags within a node
        '''
        tagSetKey = set()
        tagSetValue = set()
        nodeList = self.getElementByTag('node', self.root)
        for node in nodeList:
            childTags = self.getElementByTag('tag', node)
            for childTag in childTags:
                tagSetKey.add(childTag.get('k'))
                tagSetValue.add(childTag.get('v'))
        return tagSetKey, tagSetValue
            
    def getWayTags(self):
        '''
        Param: None
        Return: Set of available tags within a way
        '''
        tagKeySet = set()
        tagValueSet = set()
        wayList = self.getElementByTag('way', self.root)
        for way in wayList:
            childTags = self.getElementByTag('tag', way)
            for childTag in childTags:
                tagKeySet.add(childTag.get('k'))
                tagValueSet.add(childTag.get('v'))
        return tagKeySet, tagValueSet
    
    def getNodeByTag(self, tags=None):
        '''
        Param: tags as a list of string
        Return: List of nodes with a particular set of tags. For eg. nodes with tags of bus_stop.
        '''
        
        nodeObjectList = []
        nodePointList = []
        nodeList = self.getElementByTag('node', self.root)
        
        for node in nodeList:
            ref = node.get('id')
            lat = node.get('lat')
            lon = node.get('lon')

            nodePointList.append((float(lat), float(lon)))
            
            childTags = self.getElementByTag('tag', node)
            key_value_pair = {}
            for childTag in childTags:
                key = childTag.get('k')
                value = childTag.get('v')
                key_value_pair[key] = value
                #print (key, value)
            
            nodeObject = Node(ref, lat, lon, key_value_pair)
            #print nodeObject
            nodeObjectList.append(nodeObject)
        return nodeObjectList, nodePointList
    
    def getWayByTag(self):
        '''
        Return: List of ways with a particular set of tags. For eg. ways with tags of highway
        '''
        
        wayList = self.getElementByTag('way', self.root)
        wayObjectList = []
        for way in wayList:
            
            ref = way.get('id')
            #Save list of nodes making up a way
            childTags = self.getElementByTag('nd', way)
            lst = []
            for childTag in childTags:
                if childTag.get('ref') is not None:
                    lst.append(childTag.get('ref'))
                    
            #Save list of key value pair for each way
            childTags = self.getElementByTag('tag', way)
            key_value_pairs = {}
            for childTag in childTags:
                key = childTag.get('k')
                value = childTag.get('v')
                key_value_pairs[key] = value
                #print (key, value)
            
            #Create Way Object
            wayObject = Way(ref, lst, key_value_pairs)
            wayObjectList.append(wayObject)
            
        return wayObjectList
    
    def getNodeHash(self):
        '''
        Param: None
        Return: Dictionary of node references to node objects
        '''
        
        nodeList = self.getNodeByTag()
        nodeHash = {}
        for node in nodeList:
            nodeHash[node.ref] = node
        return nodeHash
    
    def generateAdjacencyList(self, wayList):
        '''
        Param: list of ways where each way is Way object
        Return: Dictionary as adjacency list
        '''
        
        adjacencyList = {}
        for way in wayList:
            if 'highway' not in way.dict.keys() :#or way.dict['highway'] != 'footway':
                continue
            nodes_refs = way.nodes
            for i in range(1, len(nodes_refs)):
                node_i = nodes_refs[i-1]
                node_j = nodes_refs[i]
                nodeSet = set(adjacencyList.keys())
                if node_i not in nodeSet and node_j not in nodeSet:
                    adjacencyList[node_i] = [node_j]
                    adjacencyList[node_j] = [node_i]
                elif node_i not in nodeSet and node_j in nodeSet:
                    adjacencyList[node_i] = [node_j]
                    adjacencyList[node_j].append(node_i)
                elif node_i in nodeSet and node_j not in nodeSet:
                    adjacencyList[node_i].append(node_j)
                    adjacencyList[node_j] = [node_i]
                else:
                    adjacencyList[node_i].append(node_j)
                    adjacencyList[node_j].append(node_i)
        return adjacencyList
        
class Way(object):
    
    def __init__(self, ref, nd, key_value_pairs):
        self.ref = ref
        self.nodes = nd
        self.dict = key_value_pairs

class Node(object):
    
    def __init__(self, ref, lat, lon, key_value_pairs):
        self.ref = ref
        self.lat = float(lat)
        self.lon = float(lon)
        self.dict = key_value_pairs

    def __str__(self):
        return "ref %s \nlat %f \nlon %f \ndict %s" % (self.ref, self.lat, self.lon, self.dict)

def closest_node(node, nodes):
    #print nodes
    #print node
    nodes = np.asarray(nodes)
    dist_2 = np.sum((nodes - node)**2, axis=1)
    return np.argmin(dist_2)

def get_information_from_local_db(latitude, longitude):
    global nodePointList
    global nodeObjectList
    
    road_node = closest_node((latitude, longitude), nodePointList)
    print nodeObjectList[road_node]

def get_information_from_db(latitude, longitude):
    global apioverpy
    global call_gmaps_nearest_roads

    if call_gmaps_nearest_roads:
       [latitude, longitude] = gmaps_nearest_roads(latitude, longitude)

    delta = 0.0001
    query = """way(%f,%f,%f,%f);out body;""" % (latitude-delta,longitude-delta,latitude+delta,longitude+delta)
    print query

    # http://wiki.openstreetmap.org/wiki/Overpass_API/Language_Guide#Streets_and_other_ways
    result = apioverpy.query(query)
    
    nodeid = 1
    maxspeed = -1
    maxheight = -1
    lanes = -1
    roadname = ""
    oneway = ""
    surface = ""
    highway = ""
    finished = True

    for way in result.ways:
        print("Name: %s" % way.tags.get("name", "n/a"))
        for tag in way.tags:
            tagvalue = way.tags.get(tag, "n/a")
            print (" %s: %s" % (tag, tagvalue))
            
            if tag == "name":
                roadname = str(tagvalue)
            if tag == "oneway":
                oneway = str(tagvalue)
            if tag == "lanes":
                lanes = int(tagvalue)
            if tag == "surface":
                surface = str(tagvalue)
            if tag == "highway":
                highway = str(tagvalue)

    return nodeid, latitude, longitude, maxspeed, maxheight, lanes, roadname, oneway, surface, highway, finished

def get_information(req):

    rospy.wait_for_service('transfrom_map_to_gps')
    try:
        transformer_service = rospy.ServiceProxy('transfrom_map_to_gps', TransformMapToGps)
        resp = transformer_service(req.map_x, req.map_y)
        return get_information_from_db(resp.latitude, resp.longitude)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def information_server():
    global nodePointList
    global nodeObjectList
    global call_gmaps_nearest_roads

    rospy.init_node('information_server')
    s = rospy.Service('get_information_of_gps', GetInformation, get_information)
    fname_osm_map = rospy.get_param('~fname_osm_map', str("/home/fnaser/map.osm"))
    call_gmaps_nearest_roads = rospy.get_param('~gmaps_nearest_roads', bool(True))

    osmmap = Map(fname_osm_map)
    [nodeObjectList, nodePointList] = osmmap.getNodeByTag()

    print ("Ready to get information from database.")
    rospy.spin()

if __name__ == "__main__":
    global apioverpy

    apioverpy = overpy.Overpass()
    information_server()

