#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from mapping_robot.srv import GetLastReadingLeft, GetLastReadingRight
 
GPIO.setmode(GPIO.BCM)

left_wheel = 23
right_wheel = 24

GPIO.setup(left_wheel, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(right_wheel, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

deltaLeft = 0
deltaRight = 0

def left_wheel_int(channel):  
    global lastReadLeft
    global deltaLeft
    current = rospy.Time.now()
    deltaLeft = current - lastReadLeft
    lastReadLeft = current
    rospy.loginfo(deltaLeft)
  
def right_wheel_int(channel):  
    global lastReadRight
    global deltaRight
    current = rospy.Time.now()
    deltaRight = current - lastReadRight
    lastReadRight = current
    rospy.loginfo(deltaRight)

GPIO.add_event_detect(left_wheel, GPIO.RISING, callback=left_wheel_int, bouncetime=5)
GPIO.add_event_detect(right_wheel, GPIO.RISING, callback=right_wheel_int, bouncetime=5)

def handle_get_last_reading_left(req):
    global lastReadLeft
    global deltaLeft
    current = rospy.Time.now()
    if current.to_sec() - lastReadLeft.to_sec() < 0.1:
        rospy.loginfo(deltaLeft)
        return deltaLeft
    else:
        rospy.loginfo(0)
        return rospy.Time()

def handle_get_last_reading_right(req):
    global lastReadRight
    global deltaRight
    current = rospy.Time.now()
    if current.to_sec() - lastReadRight.to_sec() < 0.1:
        rospy.loginfo(deltaRight)
        return deltaRight
    else:
        rospy.loginfo(0)
        return rospy.Time()

if __name__ == '__main__':
    rospy.init_node('RPM', anonymous=False)
    lastReadLeft = rospy.Time.now()
    lastReadRight = rospy.Time.now()
    s1 = rospy.Service('get_last_reading_left', GetLastReadingLeft, handle_get_last_reading_left)
    s2 = rospy.Service('get_last_reading_right', GetLastReadingRight, handle_get_last_reading_right)
    rospy.spin()
