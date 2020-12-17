#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist

#uncomment commands to view camera


rospy.init_node('Camera')

cap = cv2.VideoCapture(0)

#cv2.namedWindow('sliders')

# HSV values to determine lanes
lowH = 0
highH = 39
lowS = 50
highS = 255
lowV = 143
highV = 255

#slider to help find hsv values

#cv2.createTrackbar('lowH', 'sliders', lowH, 179, callback)
#cv2.createTrackbar('highH', 'sliders', highH, 179, callback)

#cv2.createTrackbar('lowS', 'sliders', lowS, 255, callback)
#cv2.createTrackbar('highS', 'sliders', highS, 255, callback)

#cv2.createTrackbar('lowV', 'sliders', lowV, 255, callback)
#cv2.createTrackbar('highV', 'sliders', highV, 255, callback)

#publisher inititalization
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
move=Twist()
rate=rospy.Rate(60)
while not rospy.is_shutdown():
    ret, frame = cap.read()
    #frame = cv2.imread('image.png')

    # get trackbar positions
    #lowH = cv2.getTrackbarPos('lowH', 'sliders')
    #highH = cv2.getTrackbarPos('highH', 'sliders')
    #lowS = cv2.getTrackbarPos('lowS', 'sliders')
    #highS = cv2.getTrackbarPos('highS', 'sliders')
    #lowV = cv2.getTrackbarPos('lowV', 'sliders')
    #highV = cv2.getTrackbarPos('highV', 'sliders')

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([lowH, lowS, lowV])
    higher = np.array([highH, highS, highV])
    #for centroid
    mask = cv2.inRange(hsv, lower, higher)
    h,w,d= frame.shape
    M=cv2.moments(mask)
    #centroid calculation and publisher
    #calculates center of mask and outputs difference with center of screen
    #publishes difference where driver code interprets data
    if M['m00'] != 0:
           cx= int(M['m10']/M['m00'])
           cy= int(M['m01']/M['m00'])
           #cv2.circle(frame, (cx, cy), 20, (0,0,255), -1)
           err=cy-h/2
           move.linear.x=0.1
           move.angular.z=-float(err)/100
           cmd_vel_pub.publish(move)
	   
    else:
           cx, cy=0, 0
	   
    rate.sleep()

    #cv2.imshow('frame', frame)
    #cv2.imshow('mask', mask)
    

    key = cv2.waitKey(100) & 0xFF
    if key == ord('q'):
        break

cap.release()
#cv2.destroyAllWindows()

