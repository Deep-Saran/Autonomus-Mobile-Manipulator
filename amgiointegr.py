#!/usr/bin/env python

import rospy
import roslib
import tf
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2,sqrt,pow

import cv2
import cv_bridge
import numpy as np
import matplotlib.pyplot as plt


x = 0.0
y = 0.0 
theta = 0.0
def callback(data):
    global x
    global y
    global theta
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    
    
    rot_q = data.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w]) 


def main():
    global x
    global y
    global theta
    rospy.init_node("mynode", anonymous=True)
    sub=rospy.Subscriber('/odom',Odometry,callback)
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(50)
    #cap= cv2.VideoCapture(0)
    #ret, img = cap.read()
    #cap.release()

    img = cv2.imread('circle.png')


# cut out excessive parts of image here
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('frame',imgGray)
    ret, thresh = cv2.threshold(imgGray, 127, 255, 0)
    image, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    largest = 0
    indexLargest = 0
    for i in xrange(0, len(contours)):
   	area = cv2.contourArea(contours[i])
    	if area > largest:
        	largest = area
        	indexLargest = i
   # print "array value: %d  array size: %d contour area: %d"% (i,contours[i].size,area)
    largestContour = contours.pop(indexLargest)
    secondLargest = 0
    indexSecondLargest = 0
    for i in xrange(0, len(contours)):
    	area = cv2.contourArea(contours[i])
        if area > secondLargest:
            secondLargest = area
            indexSecondLargest = i
    #print "array value: %d  array size: %d contour area: %d"% (i,contours[i].size,area)

    contourArray = contours[indexSecondLargest]#46
    print "second largest index: %d  "% (indexSecondLargest)
    cArray = contourArray[:, 0]
    picx = cArray[:, 0]
    for point in picx:
       print "%d ," % (point)
#print(x)

    print("y array/n")

    picy = cArray[:, 1]
#print(y)

    plt.plot(picx, picy)
    plt.show()
    n=len(picx)
    i=0
    for i in xrange(n):

    	p= (picx[i]-picx[0])
    	q= (picy[i]-picy[0])
    	print("x")
	print(p)
	print("y")
	print(q)
    	print("scale x")
	p=p/200.0
	q=q/200.0	
	print(p)
	print("scale y")
	print(q)
    	while not rospy.is_shutdown():
		speed = Twist()
    		goal = Point()
		goal.x=p
		goal.y=q
        	inc_x = goal.x -x
    		inc_y = goal.y -y
 		
    		angle_to_goal = atan2(inc_y, inc_x)
        	euclidean_distance = sqrt(pow((goal.x - round(x,6)), 2) + pow((goal.y - round(y,6)), 2))
	
    		if abs(angle_to_goal - theta) > 0.1:
			speed.linear.x = 0
			speed.angular.z=1.5*(angle_to_goal - theta)
			pub.publish(speed)          

    		else:
			speed.linear.x = 1*euclidean_distance
			speed.angular.z=0
			pub.publish(speed)
        		if(round(euclidean_distance,1)==0):
				speed.linear.x = 0
				speed.angular.z=0   
        		        pub.publish(speed)
				break
   
    #rospy.spin()

if __name__=="__main__":
    main()
