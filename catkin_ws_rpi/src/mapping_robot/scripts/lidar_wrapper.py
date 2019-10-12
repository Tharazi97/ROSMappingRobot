#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from mapping_robot.msg import Point

# We are using this subscriber publiher class to ease using publish and subscribe at the same time
class SubPub:
	# In the callback function we invoke all peripherals wrapper functions
	def callback(self, msg):
		if msg.data == 1:
			for i in range(180):
				for j in range(180):
					point = Point()
					point.distance = i + j
					# TODO: Michals peripherals functions
					#point.distance = lidar_read()
					#angle = mpu_read()
					#point.roll = angle.roll
					#point.pitch = angle.pitch
					#point.yaw = angle.yaw
					self.pub.publish(point)
					#move_servo(i, j)

	def __init__(self):	
		# Inform master that we will be subscribing Bool msgs on the create_map toppic
		self.sub = rospy.Subscriber('create_map', Bool, self.callback)

		# Inform master that we will be publishing Point msgs on the measured_points toppic.
		self.pub = rospy.Publisher("measured_points", Point, queue_size=100)

if __name__ == '__main__':
	rospy.init_node('lidar_wrapper', anonymous=True)
	handle = SubPub()
	rospy.spin()
