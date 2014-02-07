#!/usr/bin/env python
import roslib; roslib.load_manifest('entity_storage')
from entity_storage.srv import *
from entity_storage.msg import *
import rospy


def get_coordinates(name):
    rospy.wait_for_service('get_entity_coordinates')
    try:
        get_coords = rospy.ServiceProxy('get_entity_coordinates', entity_coordinates)
        resp1 = get_coords(name)
        return resp1.coordinate
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def usage():
    return "%s [entity_name]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        entity_name = sys.argv[1]
    else:
        print usage()
        sys.exit(1)
    print "Requesting coordinates for %s" % entity_name
    coordinate = get_coordinates(entity_name)
    print "%s: X:%s  Y:%s  Theta:%s" % (entity_name, coordinate.x, coordinate.y, coordinate.theta)