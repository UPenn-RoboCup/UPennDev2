#!/usr/bin/env python
execfile("../fiddle.py")

# Now add a ROS subscriber
import string
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion, Twist

wcm = lg.wcm
mcm = lg.mcm

def callback(data):
    rospy.loginfo(rospy.get_name())
		print(data)
		vel = mcm.get_walk_vel()
		vel[1L] = float(linear.x)
		vel[2L] = float(linear.x)
		vel[3L] = float(angular.z)
		mcm.set_walk_vel( vel )

def listener():
    rospy.init_node('ambassor')
    rospy.Subscriber("/cmd_vel", Twist, callback)

		try:
        # Formulate the odometry
				msg = Odometry()
				msg.header.stamp = rospy.Time.now()
				msg.header.frame_id = '/odom'
				msg.child_frame_id = '/base_footprint'
				pose = wcm.get_robot_odom()
				msg.pose.pose.position = Point(pose[1L], pose[2L], pose[3L])
				msg.pose.pose.orientation = Quaternion(1,0,0,0)
				pub.publish(msg)
				print("Published Odometry!")
				print(msg)

    except rospy.ROSInterruptException:
        pass

		rospy.sleep(1.0)
    #rospy.spin()

if __name__ == '__main__':
    listener()
