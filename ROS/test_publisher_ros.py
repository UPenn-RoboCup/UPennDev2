#!/usr/bin/env python
execfile("../fiddle.py")

import rospy
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('talker')
    while not rospy.is_shutdown():
        str = "hello world %s" % rospy.get_time()
        rospy.loginfo(str)
        pub.publish(String(str))
				# Formulate the odometry
				msg = Odometry()
				# Sleep a bit
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
