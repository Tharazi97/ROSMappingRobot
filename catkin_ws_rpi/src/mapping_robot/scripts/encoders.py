#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from mapping_robot.srv import GetEncoder, ChangeDir
from mapping_robot.msg import Encoder

rospy.init_node('RPM', anonymous=False)
 
GPIO.setmode(GPIO.BCM)

left_wheel = 27
right_wheel = 26

GPIO.setup(left_wheel, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(right_wheel, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

directionL = True
directionR = True

counterL = 0
counterR = 0

lastTimeL = 0
lastTimeR = 0

averageL = 0
averageR = 0

def left_wheel_int(channel):
    global lastTimeL
    global averageL
    global counterL
    currTimeL = rospy.Time.now().to_sec()
    deltaL = currTimeL - lastTimeL
    if directionL:
        averageL = ( ( averageL * counterL ) / ( counterL + 1 ) ) + ( deltaL / ( counterL + 1 ) )
    else:
        averageL = ( ( averageL * counterL ) / ( counterL + 1 ) ) - ( deltaL / ( counterL + 1 ) )
    counterL += 1
    lastTimeL = currTimeL    
  
def right_wheel_int(channel):  
    global lastTimeR
    global averageR
    global counterR
    currTimeR = rospy.Time.now().to_sec()
    deltaR = currTimeR - lastTimeR
    if directionR:
        averageR = ( ( averageR * counterR ) / ( counterR + 1 ) ) + ( deltaR / ( counterR + 1 ) )
    else:
        averageR = ( ( averageR * counterR ) / ( counterR + 1 ) ) - ( deltaR / ( counterR + 1 ) )
    counterR += 1
    lastTimeR = currTimeR  


GPIO.add_event_detect(left_wheel, GPIO.BOTH, callback=left_wheel_int, bouncetime=5)
GPIO.add_event_detect(right_wheel, GPIO.BOTH, callback=right_wheel_int, bouncetime=5)

def handle_GetEncoderL(req):
    global averageL
    global lastTimeL
    global counterL
    enco = Encoder()
    enco.delta = averageL
    enco.timeStamp = lastTimeL
    averageL = 0.0
    counterL = 0.0
    return enco

def handle_GetEncoderR(req):
    global averageR
    global lastTimeR
    global counterR
    enco = Encoder()
    enco.delta = averageR
    enco.timeStamp = lastTimeR
    averageR = 0.0
    counterR = 0.0
    return enco
    
def handle_ChangeDirL(req):
    global directionL
    directionL = req.data
    return True

def handle_ChangeDirR(req):
    global directionR
    directionR = req.data
    return True

if __name__ == '__main__':
    rospy.sleep(1.0)
    lastTimeL = rospy.Time.now().to_sec()
    lastTimeR = rospy.Time.now().to_sec()
    sEncoderL = rospy.Service('GetEncoderL', GetEncoder, handle_GetEncoderL)
    sEncoderR = rospy.Service('GetEncoderR', GetEncoder, handle_GetEncoderR)
    sDirL = rospy.Service('ChangeDirL', ChangeDir, handle_ChangeDirL)
    sDirR = rospy.Service('ChangeDirR', ChangeDir, handle_ChangeDirR)
    rospy.spin()
