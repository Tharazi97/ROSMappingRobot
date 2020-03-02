#!/usr/bin/env python
import rospy
import std_msgs.msg
import wiringpi
import serial
import math
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32

if __name__ == '__main__':

	# Number of laser reading per half cycle
	num_readings = 180

	rospy.init_node('Scanner3D', anonymous=False)
	pub = rospy.Publisher('map3D', PointCloud, queue_size=10)

	#SETUP
	wiringpi.wiringPiSetupGpio()

	horizontalServo = 12
	verticalServo = 13

	# Set GPIO18 as vertical servo and GPIO12 as horizontal servo
	wiringpi.pinMode(horizontalServo, wiringpi.GPIO.PWM_OUTPUT)
	wiringpi.pinMode(verticalServo, wiringpi.GPIO.PWM_OUTPUT)

	# Set PWM to mark:space mode
	wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)

	# Set the dvider of PWM clock to 96 and range of counting to 4000 so it gives 50 Hz frequency
	wiringpi.pwmSetClock(96)
	wiringpi.pwmSetRange(4000)

	lid = serial.Serial(port = "/dev/ttyAMA0", baudrate = 115200, bytesize = serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, timeout = 2)

	lid.write(bytearray([90, 5, 5, 1, 101])) #centimeters
	lid.write(bytearray([90, 6, 3, 250, 0, 93])) #250 Hz

	rospy.sleep(0.4)

	while not rospy.is_shutdown():

		scan3D = PointCloud()
		scan3D.header.seq = 0
		scan3D.header.stamp = rospy.Time.now()
		scan3D.header.frame_id = "laser_frame"

		distance_channel = ChannelFloat32()
		distance_channel.name = "distance"


		wiringpi.pwmWrite(horizontalServo, 108)
		wiringpi.pwmWrite(verticalServo, 93)
		lid.reset_input_buffer()

		for i in range(num_readings/4):    #
			if rospy.is_shutdown():
				break

			lid.reset_input_buffer()
			for j in range(num_readings):
				if rospy.is_shutdown():
					break

				s_data1 = lid.read(1)
				s_data2 = lid.read(1)
				while s_data1 <> 'Y' or s_data2 <> 'Y':
					if rospy.is_shutdown():
						break
					s_data1 = s_data2
					s_data2 = lid.read(1)

				serial_data = list(lid.read(4))
				if len(serial_data) == 4:
					distance = (ord(serial_data[0]) + (ord(serial_data[1]) << 8))/100.0
					if distance < 12:
						point = Point32()
						point.x = distance * math.cos(math.radians(i*(num_readings/180))) * math.cos(math.radians(j*num_readings/180 - 90))
						point.y = distance * math.cos(math.radians(i*(num_readings/180))) * math.sin(math.radians(j*num_readings/180 - 90))
						point.z = distance * math.sin(math.radians(i*(num_readings/180)))
						scan3D.points.append(point)
						distance_channel.values.append(distance)
				# Move to the servo to the next point
				wiringpi.pwmWrite(horizontalServo, 108+(j*372/num_readings))

			wiringpi.pwmWrite(horizontalServo, 108)
			wiringpi.pwmWrite(verticalServo, 480 - (i * 397/num_readings))
			rospy.sleep(0.6)

			if rospy.is_shutdown():
	                        break

			lid.reset_input_buffer()
			for j in range(num_readings):
				if rospy.is_shutdown():
					break

				s_data1 = lid.read(1)
				s_data2 = lid.read(1)
				while s_data1 <> 'Y' or s_data2 <> 'Y':
					if rospy.is_shutdown():
						break
					s_data1 = s_data2
					s_data2 = lid.read(1)

				serial_data = list(lid.read(4))
				if len(serial_data) == 4:
					distance = (ord(serial_data[0]) + (ord(serial_data[1]) << 8))/100.0
					if distance < 12:
						point = Point32()
						point.x = distance * math.cos(math.radians(i*(num_readings/180))) * math.cos(math.radians(j*num_readings/180 + 90))
						point.y = distance * math.cos(math.radians(i*(num_readings/180))) * math.sin(math.radians(j*num_readings/180 + 90))
						point.z = distance * math.sin(math.radians(i*(num_readings/180)))
						scan3D.points.append(point)
						distance_channel.values.append(distance)
				# Move to the servo to the next point
				wiringpi.pwmWrite(horizontalServo, 108+(j*372/num_readings))

			wiringpi.pwmWrite(horizontalServo, 108)
			wiringpi.pwmWrite(verticalServo, 93 + (i * 397/num_readings))
			rospy.sleep(0.6)

		scan3D.channels.append(distance_channel)
		pub.publish(scan3D)
		rospy.sleep(2.0)
