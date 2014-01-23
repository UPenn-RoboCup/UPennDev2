#!/usr/bin/env python
execfile("../fiddle.py")

# Now add a ROS subscriber
import rospy
from std_msgs.msg import String
import string

def callback(data):
    rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)
    stuff = data.data.split(" ")
    t = string.atof(stuff[2])
    print("Time",t)
    lua.execute("mcm.set_status_t({: f})".format(t))

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
