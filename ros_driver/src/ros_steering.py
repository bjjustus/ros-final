#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Twist
from adafruit_servokit import ServoKit
import adafruit_motor.servo

msg=Twist()
#initialize kit
kit = ServoKit(channels=16) 

def callback(msg):
    z=msg.angular.z
    #turns error value into angle
    steering = 13*z+90
    print(steering)
    #steers car
    kit.servo[1].angle = steering
def driver():
    #subscriber gets cmd_vel values from open cv file
    rospy.init_node('car')
    rospy.Subscriber('/cmd_vel', Twist, callback)
    #outputs constants throttle
    kit.continuous_servo[2].throttle = 0.1
    rospy.spin()
    
	
driver()


