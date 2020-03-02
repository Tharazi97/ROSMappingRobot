#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def talker():
	pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
	rospy.init_node('initpose', anonymous=False)

	initpose = PoseWithCovarianceStamped()

	initpose.header.stamp = rospy.Time.now()
	initpose.header.frame_id = "map"
	initpose.header.seq = 0

	initpose.pose.pose.position.x = 0
	initpose.pose.pose.position.y = 0
	initpose.pose.pose.position.z = 0

	initpose.pose.pose.orientation.x = 0
	initpose.pose.pose.orientation.y = 0
	initpose.pose.pose.orientation.z = 0
	initpose.pose.pose.orientation.w = 1


	initpose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
	pub.publish(initpose)

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
