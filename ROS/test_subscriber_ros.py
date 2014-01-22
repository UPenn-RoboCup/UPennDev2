#!/usr/bin/env python

# Very important for linux, via:
# http://stackoverflow.com/questions/8361437/linker-error-lunatic-python-lua-requiresocket-undefined-symbol-lua-getme
import sys

if sys.platform != "darwin":
	import DLFCN
	sys.setdlopenflags(DLFCN.RTLD_NOW | DLFCN.RTLD_GLOBAL)

# Import luantic-python
import lua
# Have all the framework files/Modules available
lua.execute('dofile"fiddle.lua"')
# Have the globals if needed...
lg = lua.globals()
require = lua.require

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
