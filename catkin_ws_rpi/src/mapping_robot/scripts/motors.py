#!/usr/bin/env python

import rospy
import wiringpi
from mapping_robot.msg import MotorPowers

wiringpi.wiringPiSetupGpio()

leftMotorPWM = 13
leftMotorIn1 = 5
leftMotorIn2 = 6

rightMotorPWM = 19
rightMotorIn1 = 16
rightMotorIn2 = 20

# Set GPIO13 as left motor pwm and GPIO19 as right motor
wiringpi.pinMode(leftMotorPWM, wiringpi.GPIO.PWM_OUTPUT)
wiringpi.pinMode(rightMotorPWM, wiringpi.GPIO.PWM_OUTPUT)

wiringpi.pinMode(leftMotorIn1, 1)
wiringpi.pinMode(leftMotorIn2, 1)

wiringpi.pinMode(rightMotorIn1, 1)
wiringpi.pinMode(rightMotorIn2, 1)

# Set PWM to mark:space mode
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)

# Set the dvider of PWM clock to 96 and range of counting to 4000 so it gives 50 Hz frequency
wiringpi.pwmSetClock(96)
wiringpi.pwmSetRange(4000)

def callback(data):
    if data.left < 0:
        ## set pin 1 high, 2 low
        wiringpi.digitalWrite(leftMotorIn1, 1)
        wiringpi.digitalWrite(leftMotorIn2, 0)
        wiringpi.pwmWrite(leftMotorPWM, data.left*(-10))
    elif data.left > 0:
        ## set pin 1 low, 2 high
        wiringpi.digitalWrite(leftMotorIn1, 0)
        wiringpi.digitalWrite(leftMotorIn2, 1)
        wiringpi.pwmWrite(leftMotorPWM, data.left*10)
    else:
        ## set both high to brake?
        wiringpi.digitalWrite(leftMotorIn1, 1)
        wiringpi.digitalWrite(leftMotorIn2, 1)
        wiringpi.pwmWrite(leftMotorPWM, 0)
    
    if data.right < 0:
        ## set pin 1 high, 2 low
        wiringpi.digitalWrite(rightMotorIn1, 1)
        wiringpi.digitalWrite(rightMotorIn2, 0)
        wiringpi.pwmWrite(rightMotorPWM, data.right*(-10))
    elif data.right > 0:
        ## set pin 1 low, 2 high
        wiringpi.digitalWrite(rightMotorIn1, 0)
        wiringpi.digitalWrite(rightMotorIn2, 1)
        wiringpi.pwmWrite(rightMotorPWM, data.right*10)
    else:
        ## set both high to brake?
        wiringpi.digitalWrite(rightMotorIn1, 1)
        wiringpi.digitalWrite(rightMotorIn2, 1)
        wiringpi.pwmWrite(rightMotorPWM, 0)
    

if __name__ == '__main__':
    rospy.init_node('motors', anonymous=False)

    rospy.Subscriber("motor_powers", MotorPowers, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
