#!/usr/bin/env python

import sys
import time
import wiringpi
import serial
import rospy
from std_msgs.msg import Bool
from mapping_robot.msg import Point

lid = serial.Serial(port=None,
					baudrate=115200,
					bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
					stopbits=serial.STOPBITS_ONE,
					timeout=2)

# use 'GPIO naming'
wiringpi.wiringPiSetupGpio()

# set #18 to be a PWM output
wiringpi.pinMode(18, wiringpi.GPIO.PWM_OUTPUT)
wiringpi.pinMode(12, wiringpi.GPIO.PWM_OUTPUT)

# set the PWM mode to milliseconds stype
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)

# divide down clock
wiringpi.pwmSetClock(192)  # 50Hz servo
wiringpi.pwmSetRange(2000)
# We are using this subscriber publisher class to ease using publish and subscribe at the same time
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

	# def lidar_read(self):
	# 	measure_dist = [90, 4, 4, 98]
	# 	delay_period = 0.1
	# 	lid.write(measure_dist)
	# 	time.sleep(delay_period)
	# 	serial_data = lid.read(size=9)
	# 	if int.from_bytes(serial_data(1), byteorder='big') == 89 and int.from_bytes(serial_data(2), byteorder='big') == 89:
	# 		dist_H = serial_data(4)
	# 		dist_L = serial_data(3)
	# 		dist = dist_H + dist_L
	# 		return(dist)

	def move_servo(self,i, j):
		wiringpi.pwmWrite(18, i*(5/9))
		wiringpi.pwmWrite(12, j*(5/9))	

	def __init__(self):	
		# Inform master that we will be subscribing Bool msgs on the create_map toppic
		self.sub = rospy.Subscriber('create_map', Bool, self.callback)

		# Inform master that we will be publishing Point msgs on the measured_points toppic.
		self.pub = rospy.Publisher("measured_points", Point, queue_size=100)

if __name__ == '__main__':
	rospy.init_node('lidar_wrapper', anonymous=True)
	handle = SubPub()
	rospy.spin()
