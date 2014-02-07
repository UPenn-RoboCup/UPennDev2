#!/usr/bin/env python
execfile("../fiddle.py")

# Now add a ROS subscriber
import rospy
from std_msgs.msg import String
# Add the data structure definition
from datamatrix_finder.msg import Datamatrix
# string converter if needed...
import string

# Globally, get the right rcm structs
# rcm is ROS cm
rcm = lg.rcm

def data_matrix_cb(data):
    rospy.loginfo('\n'+rospy.get_name()+" received: "+data.message)
    #print("Translation: ",data.translation,type(data.translation))
    #print("Rot: ",data.rotation,type(data.rotation))
    # Save the translation
    trans = rcm.get_datamatrix_translation()
    for i,v in enumerate(data.translation):
        trans[long(i+1)] = float(v)
    rcm.set_datamatrix_translation(trans)
    # Save the rotation
    rot = rcm.get_datamatrix_rotation()
    for i,v in enumerate(data.rotation):
        rot[long(i+1)] = float(v)
    rcm.set_datamatrix_rotation(rot)

def listener():
    rospy.init_node('steve_listen')
    rospy.Subscriber("/datamatrix", Datamatrix, data_matrix_cb)
    rospy.spin()

if __name__ == '__main__':
    listener()
