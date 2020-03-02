#!/usr/bin/env python

import rospy
import wiringpi
import serial
import std_msgs.msg
import math

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import Int16

from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32

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

isMoving = False
scan3DAngle = 0


def callbackMove(msg):
	global isMoving
	isMoving = True

def callback3DScan(msg):
	global scan3DAngle
	scan3DAngle = msg.data

if __name__ == '__main__':
	global isMoving
	global scan3DAngle
	sequence = 0
	rospy.init_node('laser_scan_publisher', anonymous=False)

	# Inform master that we will be publishing LaserScan msgs on the scan toppic.
	scan_pub = rospy.Publisher("scan", LaserScan, queue_size=100)
	scan3D_pub = rospy.Publisher('map3D', PointCloud, queue_size=10)
	rospy.Subscriber("is_moving", Bool, callbackMove)
	rospy.Subscriber("make_3DScan", Int16, callback3DScan)

	#scan3D = PointCloud()
	#scan3D.header.seq = sequence
	#sequence = sequence + 1
	#scan3D.header.stamp = rospy.Time.now()
	#scan3D.header.frame_id = "laser_frame"

	#distance_channel = ChannelFloat32()
	#distance_channel.name = "distance"

	#scan3D.channels.append(distance_channel)

	# Number of laser reading per half cycle
	num_readings2D = 360
	num_readings3D = 180

	# Rate of 1 full path of lidar 
	#rate = rospy.Rate(0.3) # 0.15hz

	# Way the lidar is spinning
	clockwise = False

	lid.write(bytearray([90, 5, 5, 1, 101]))
	lid.write(bytearray([90, 6, 3, 250, 0, 93]))

	wiringpi.pwmWrite(horizontalServo, 480)
        wiringpi.pwmWrite(verticalServo, 470)
	rospy.sleep(0.4)

	wiringpi.pwmWrite(horizontalServo, 108)
        wiringpi.pwmWrite(verticalServo, 95)
	rospy.sleep(0.4)

	while not rospy.is_shutdown():

		if scan3DAngle == 0:
			# Create laser scan msg with parameters provided below
			scan = LaserScan()
			scan.header.stamp = rospy.Time.now()
			scan.header.frame_id = "laser_frame"
			scan.angle_min = -1.57
			scan.angle_max = 4.71
			scan.angle_increment = 3.14 / num_readings2D
			# scan.scan_time = 1/0.3
			# scan.time_increment = 0.01
			scan.range_min = 0.1
			scan.range_max = 12.0

			wiringpi.pwmWrite(horizontalServo, 108)
			wiringpi.pwmWrite(verticalServo, 95)
			lid.reset_input_buffer()
			for i in range(num_readings2D):
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
					scan.ranges.append(distance)
					scan.intensities.append(ord(serial_data[2]) + (ord(serial_data[3]) << 8))
				else:
					# If the lenght of the data read was incorrect, expect that the whole frame is incorrect and put the data outside the range limit of laser at the end of the list
					scan.ranges.append(0)
					scan.intensities.append(0)
				# Move to the servo to the next point
				wiringpi.pwmWrite(horizontalServo, 108+(i*186/180))
			wiringpi.pwmWrite(horizontalServo, 108)
			wiringpi.pwmWrite(verticalServo, 470)
			rospy.sleep(0.4)

			lid.reset_input_buffer()
			for i in range(num_readings2D):
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
					scan.ranges.append(distance)
					scan.intensities.append(ord(serial_data[2]) + (ord(serial_data[3]) << 8))
				else:
					# If the lenght of the data read was incorrect, expect that the whole frame is incorrect and put the data outside the range limit of laser at the end of the list
					scan.ranges.append(0)
					scan.intensities.append(0)
				# Move to the servo to the next point
				wiringpi.pwmWrite(horizontalServo, 108+(i*186/180))
			wiringpi.pwmWrite(horizontalServo, 108)
			wiringpi.pwmWrite(verticalServo, 95)

			scan.scan_time = rospy.Time.now().to_sec() - scan.header.stamp.to_sec()
			scan.time_increment = (rospy.Time.now().to_sec() - scan.header.stamp.to_sec())/(2*num_readings2D)
			# Publish created LaserRead scan
			if isMoving == False:
				scan_pub.publish(scan)

			isMoving = False
			rospy.sleep(0.4)
			#rate.sleep()
		else:
			scan3D = PointCloud()
	                scan3D.header.seq = sequence
			sequence = sequence + 1
	                scan3D.header.stamp = rospy.Time.now()
	                scan3D.header.frame_id = "laser_frame"

	                distance_channel = ChannelFloat32()
	                distance_channel.name = "distance"


	                wiringpi.pwmWrite(horizontalServo, 108)
	                wiringpi.pwmWrite(verticalServo, 95)
	                lid.reset_input_buffer()

	                for i in range(scan3DAngle):    #
	                        if rospy.is_shutdown():
	                                break

	                        lid.reset_input_buffer()
	                        for j in range(num_readings3D):
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
	                                                point.x = distance * math.cos(math.radians(i*(num_readings3D/180))) * math.cos(math.radians(j*num_readings3D/180 - 90))
	                                                point.y = distance * math.cos(math.radians(i*(num_readings3D/180))) * math.sin(math.radians(j*num_readings3D/180 - 90))
	                                                point.z = distance * math.sin(math.radians(i*(num_readings3D/180)))
	                                                scan3D.points.append(point)
							#scan3D.channels[0].values.append(distance)
	                                                distance_channel.values.append(distance)
	                                # Move to the servo to the next point
	                                wiringpi.pwmWrite(horizontalServo, 108+(j*372/num_readings3D))

	                        wiringpi.pwmWrite(horizontalServo, 108)
	                        wiringpi.pwmWrite(verticalServo, 470 - (i * 375/num_readings3D))
	                        rospy.sleep(0.6)

	                        if rospy.is_shutdown():
	                                break

	                        lid.reset_input_buffer()
	                        for j in range(num_readings3D):
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
	                                                point.x = distance * math.cos(math.radians(i*(num_readings3D/180))) * math.cos(math.radians(j*num_readings3D/180 + 90))
	                                                point.y = distance * math.cos(math.radians(i*(num_readings3D/180))) * math.sin(math.radians(j*num_readings3D/180 + 90))
	                                                point.z = distance * math.sin(math.radians(i*(num_readings3D/180)))
	                                                scan3D.points.append(point)
							#scan3D.channels[0].values.append(distance)
	                                                distance_channel.values.append(distance)
	                                # Move to the servo to the next point
	                                wiringpi.pwmWrite(horizontalServo, 108+(j*372/num_readings3D))

	                        wiringpi.pwmWrite(horizontalServo, 108)
	                        wiringpi.pwmWrite(verticalServo, 95 + (i * 375/num_readings3D))
	                        rospy.sleep(0.6)

	                scan3D.channels.append(distance_channel)
	                scan3D_pub.publish(scan3D)
			#scanp2 = PointCloud2()
	                #rospy.sleep(2.0)
			scan3DAngle = 0
		#scan3D.header.seq = sequence
	        #sequence = sequence + 1
	        #scan3D.header.stamp = rospy.Time.now()
		#scan3D_pub.publish(scan3D)
