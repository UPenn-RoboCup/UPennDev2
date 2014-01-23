#!/usr/bin/env python

from entity_storage.srv import *
from entity_storage.msg import *
import rospy

entity_map = {}

def get_coordinates(req):
    print "Retrieving the coordinates for '%s'" % req.name
    try:
      coords = coordinates()
      coords.x = float(entity_map[req.name]['x'])
      coords.y = float(entity_map[req.name]['y'])
      coords.theta = float(entity_map[req.name]['theta'])
      #entity_coords = entity_coordinates()
      #entity_coords.coordinate = coords
      return entity_coordinatesResponse(coords)
    except Exception:
      return False

def storage_manager():
    initialize()
    rospy.init_node('entity_storage_manager')
    s = rospy.Service('get_entity_coordinates', entity_coordinates, get_coordinates)
    rospy.spin()

def initialize():
    print "Initializing the map"
    open_file = open('src/entity_storage/lists/list.txt','r')

    for line in open_file:
      tokens = line.split('\t')
      entity_map[tokens[0]] = {}
      for i in range(1,len(tokens)):
	coordinates = tokens[i].split(',')
	entity_map[tokens[0]]['x'] = coordinates[0]
	entity_map[tokens[0]]['y'] = coordinates[1]
	entity_map[tokens[0]]['theta'] = coordinates[2]
	
    open_file.close()
    
if __name__ == "__main__":
    storage_manager()