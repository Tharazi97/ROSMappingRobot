#!/usr/bin/env python

import rospy
import wiringpi
import serial
from sensor_msgs.msg import LaserScan

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


if __name__ == '__main__':
    rospy.init_node('laser_scan_publisher', anonymous=False)

    # Inform master that we will be publishing LaserScan msgs on the scan toppic.
    scan_pub = rospy.Publisher("scan", LaserScan, queue_size=100)
    
    # Number of laser reading per half cycle
    num_readings = 360
    
    # Rate of 1 full path of lidar 
    rate = rospy.Rate(0.15) # 0.15hz

    # Way the lidar is spinning
    clockwise = False

    while not rospy.is_shutdown():

        wiringpi.pwmWrite(horizontalServo, 108) # 0 for horizontal servo
        wiringpi.pwmWrite(verticalServo, 108) # 0 for vertical servo
        rospy.sleep(0.3)
        # Create laser scan msg with parameters provided below
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "laser_frame"
        scan.angle_min = 0
        scan.angle_max = 6.28
        scan.angle_increment = 6.28 / (num_readings * 2)
        scan.time_increment = 0.01
        scan.range_min = 0.1
        scan.range_max = 12.0

        for i in range(num_readings):
            lid.write(bytearray([90, 4, 4, 98]))
            # Store read data in list format to have easer acces to separate bytes
            serial_data = list(lid.read(9))
            if len(serial_data) == 9:
                # Check frame start
                if serial_data[0] == 'Y' and serial_data[1] == 'Y':
                    # Decode the data frame and put it at the end of LaserScan list
                    scan.ranges.append( (ord(serial_data[2]) + (ord(serial_data[3]) << 8))/100.0 )
                    scan.intensities.append(ord(serial_data[4]) + (ord(serial_data[5]) << 8))
                else:
                    # If the frame start was incorrect, expect that the whole data frame is incorrect and put the data outside the range limit of laser at the end of the list
                    scan.ranges.append(20)
                    scan.intensities.append(0)
            else:
                # If the lenght of the data read was incorrect, expect that the whole frame is incorrect and put the data outside the range limit of laser at the end of the list
                scan.ranges.append(20)
                scan.intensities.append(0)
            # Move to the servo to the next point
            wiringpi.pwmWrite(horizontalServo, 108+(i*186/180))

        wiringpi.pwmWrite(horizontalServo, 108) # 0 for horizontal servo
        wiringpi.pwmWrite(verticalServo, 470) # 180 for vertical servo
        rospy.sleep(0.3)

        for i in range(num_readings):
            lid.write(bytearray([90, 4, 4, 98]))
            # Store read data in list format to have easer acces to separate bytes
            serial_data = list(lid.read(9))
            if len(serial_data) == 9:
                # Check frame start
                if serial_data[0] == 'Y' and serial_data[1] == 'Y':
                    # Decode the data frame and put it at the end of LaserScan list
                    scan.ranges.append( (ord(serial_data[2]) + (ord(serial_data[3]) << 8))/100.0 )
                    scan.intensities.append(ord(serial_data[4]) + (ord(serial_data[5]) << 8))
                else:
                    # If the frame start was incorrect, expect that the whole data frame is incorrect and put the data outside the range limit of laser at the end of the list
                    scan.ranges.append(20)
                    scan.intensities.append(0)
            else:
                # If the lenght of the data read was incorrect, expect that the whole frame is incorrect and put the data outside the range limit of laser at the end of the list
                scan.ranges.append(20)
                scan.intensities.append(0)
            # Move to the servo to the next point
            wiringpi.pwmWrite(horizontalServo, 108+(i*186/180))


        # Publish created LaserRead scan
        scan_pub.publish(scan)

        rate.sleep()

