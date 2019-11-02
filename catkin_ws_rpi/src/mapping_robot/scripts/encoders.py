#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from mapping_robot.srv import GetTicksL, GetTicksR, ChangeDir
 
GPIO.setmode(GPIO.BCM)

left_wheel = 23
right_wheel = 24

GPIO.setup(left_wheel, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(right_wheel, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

ticksL = 0
ticksR = 0

directionL = True
directionR = True

def left_wheel_int(channel):  
    global ticksL
    if directionL:
        ticksL += 1
    else:
        ticksL -= 1

    if ticksL > 32767:
        ticksL = -32768
    elif ticksL < -32768:
        ticksL = 32767
    
  
def right_wheel_int(channel):  
    global ticksR
    if directionR:
        ticksR += 1
    else:
        ticksR -= 1

    if ticksR > 32767:
        ticksR = -32768
    elif ticksR < -32768:
        ticksR = 32767


GPIO.add_event_detect(left_wheel, GPIO.BOTH, callback=left_wheel_int, bouncetime=5)
GPIO.add_event_detect(right_wheel, GPIO.BOTH, callback=right_wheel_int, bouncetime=5)

def handle_GetTicksL(req):
    global ticksL
    return ticksL

def handle_GetTicksR(req):
    global ticksR
    return ticksR
    
def handle_ChangeDirL(req):
    global directionL
    directionL = req.data

def handle_ChangeDirR(req):
    global directionR
    directionR = req.data

if __name__ == '__main__':
    rospy.init_node('RPM', anonymous=False)
    lastReadLeft = rospy.Time.now()
    lastReadRight = rospy.Time.now()
    sTicksL = rospy.Service('GetTicksL', GetTicksL, handle_GetTicksL)
    sTicksR = rospy.Service('GetTicksR', GetTicksR, handle_GetTicksR)
    sDirL = rospy.Service('ChangeDirL', ChangeDir, handle_ChangeDirL)
    sDirR = rospy.Service('ChangeDirR', ChangeDir, handle_ChangeDirR)
    rospy.spin()
