#!/usr/bin/env python

import rospy
import wiringpi
from std_msgs.msg import Bool
from mapping_robot.msg import Point

wiringpi.wiringPiSetupGpio()

# Set GPIO18 as horizontal servo and GPIO12 as vertical servo
wiringpi.pinMode(12, wiringpi.GPIO.PWM_OUTPUT)
wiringpi.pinMode(18, wiringpi.GPIO.PWM_OUTPUT)

# Set PWM to mark:space mode
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)

# Set the dvider of PWM clock to 96 and range of counting to 4000 so it gives 50 Hz frequency
wiringpi.pwmSetClock(96)
wiringpi.pwmSetRange(4000)

# We are using this subscriber publiher class to ease using publish and subscribe at the same time
class SubPub:
    def move_servo(self, i, j):
        # We need to convert degrees to the value between 0-4000
        wiringpi.pwmWrite(12, 108+(i*372/180))
        wiringpi.pwmWrite(18, 108+(j*372/180))
        rospy.sleep(0.1)

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
                    self.move_servo(j, j)

    def __init__(self):	
        # Inform master that we will be subscribing Bool msgs on the create_map toppic
        self.sub = rospy.Subscriber('create_map', Bool, self.callback)

        # Inform master that we will be publishing Point msgs on the measured_points toppic.
        self.pub = rospy.Publisher("measured_points", Point, queue_size=100)


if __name__ == '__main__':
	rospy.init_node('lidar_wrapper', anonymous=False)
	handle = SubPub()
	rospy.spin()
