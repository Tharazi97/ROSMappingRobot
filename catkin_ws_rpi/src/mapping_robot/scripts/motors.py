#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from mapping_robot.msg import MotorPowers

GPIO.setmode(GPIO.BCM)

leftMotorPWM = 23
leftMotorIn1 = 16
leftMotorIn2 = 20

rightMotorPWM = 22
rightMotorIn1 = 5
rightMotorIn2 = 6

# Set GPIO23 as left motor pwm and GPIO22 as right motor
GPIO.setup(leftMotorPWM, GPIO.OUT)
GPIO.setup(rightMotorPWM, GPIO.OUT)

# create an objects for PWM at 50 Hertz
lPWM = GPIO.PWM(leftMotorPWM, 50)
rPWM = GPIO.PWM(rightMotorPWM, 50)

# start them with 0% duty cycle
lPWM.start(0)
rPWM.start(0)

# pins to control H-bridge
GPIO.setup(leftMotorIn1, GPIO.OUT)
GPIO.setup(leftMotorIn2, GPIO.OUT)

GPIO.setup(rightMotorIn1, GPIO.OUT)
GPIO.setup(rightMotorIn2, GPIO.OUT)

def callback(data):
    if data.left < 0:
        ## set pin 1 high, 2 low	
        GPIO.output(leftMotorIn1, GPIO.HIGH)
        GPIO.output(leftMotorIn2, GPIO.LOW)
        lPWM.ChangeDutyCycle(data.left*(-1))
    elif data.left > 0:
        ## set pin 1 low, 2 high
        GPIO.output(leftMotorIn1, GPIO.LOW)
        GPIO.output(leftMotorIn2, GPIO.HIGH)
        lPWM.ChangeDutyCycle(data.left)
    else:
        ## set both high to brake?
        GPIO.output(leftMotorIn1, GPIO.HIGH)
        GPIO.output(leftMotorIn2, GPIO.HIGH)
        lPWM.ChangeDutyCycle(0)
    
    if data.right < 0:
        ## set pin 1 high, 2 low
        GPIO.output(rightMotorIn1, GPIO.HIGH)
        GPIO.output(rightMotorIn2, GPIO.LOW)
        rPWM.ChangeDutyCycle(data.right*(-1))
    elif data.right > 0:
        ## set pin 1 low, 2 high
        GPIO.output(rightMotorIn1, GPIO.LOW)
        GPIO.output(rightMotorIn2, GPIO.HIGH)
        rPWM.ChangeDutyCycle(data.right)
    else:
        ## set both high to brake?
        GPIO.output(rightMotorIn1, GPIO.HIGH)
        GPIO.output(rightMotorIn2, GPIO.HIGH)
        rPWM.ChangeDutyCycle(0)
    

if __name__ == '__main__':
    rospy.init_node('motors', anonymous=False)

    rospy.Subscriber("motor_powers", MotorPowers, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    lPWM.stop()
    rPWM.stop()
    GPIO.cleanup()
