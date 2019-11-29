#!/usr/bin/env python

import rospy
import wiringpi
import serial
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

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

def callbackMove(msg):
    global isMoving
    isMoving = True

if __name__ == '__main__':
    rospy.init_node('laser_scan_publisher', anonymous=False)

    # Inform master that we will be publishing LaserScan msgs on the scan toppic.
    scan_pub = rospy.Publisher("scan", LaserScan, queue_size=100)
    rospy.Subscriber("is_moving", Bool, callbackMove)

    # Number of laser reading per half cycle
    num_readings = 360

    # Rate of 1 full path of lidar 
    rate = rospy.Rate(1) # 0.15hz

    # Way the lidar is spinning
    clockwise = False

    lid.write(bytearray([90, 6, 3, 250, 0, 93]))

    while not rospy.is_shutdown():

        # Create laser scan msg with parameters provided below
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "laser_frame"
        scan.angle_min = -1.57
        scan.angle_max = 4.71
        scan.angle_increment = 3.14 / num_readings
        # scan.time_increment = 0.01
        scan.range_min = 0.1
        scan.range_max = 12.0

        lid.reset_input_buffer()
        wiringpi.pwmWrite(horizontalServo, 108)
        wiringpi.pwmWrite(verticalServo, 470)
        for i in range(num_readings):
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
                scan.ranges.append( (ord(serial_data[0]) + (ord(serial_data[1]) << 8))/100.0 )
                scan.intensities.append(ord(serial_data[2]) + (ord(serial_data[3]) << 8))
            else:
                # If the lenght of the data read was incorrect, expect that the whole frame is incorrect and put the data outside the range limit of laser at the end of the list
                scan.ranges.append(20)
                scan.intensities.append(0)
            # Move to the servo to the next point
            wiringpi.pwmWrite(horizontalServo, 108+(i*186/180))
        wiringpi.pwmWrite(horizontalServo, 108)
        wiringpi.pwmWrite(verticalServo, 108)
        rospy.sleep(0.4)

        lid.reset_input_buffer()
        for i in range(num_readings):
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
                scan.ranges.append( (ord(serial_data[0]) + (ord(serial_data[1]) << 8))/100.0 )
                scan.intensities.append(ord(serial_data[2]) + (ord(serial_data[3]) << 8))
            else:
                # If the lenght of the data read was incorrect, expect that the whole frame is incorrect and put the data outside the range limit of laser at the end of the list
                scan.ranges.append(20)
                scan.intensities.append(0)
            # Move to the servo to the next point
            wiringpi.pwmWrite(horizontalServo, 108+(i*186/180))
        wiringpi.pwmWrite(horizontalServo, 108)
        wiringpi.pwmWrite(verticalServo, 470)


        scan.time_increment = (rospy.Time.now().to_sec() - scan.header.stamp.to_sec())/(2*num_readings)
        # Publish created LaserRead scan
        if isMoving == False:
            scan_pub.publish(scan)

        isMoving = False
        rospy.sleep(0.4)
        rate.sleep()
