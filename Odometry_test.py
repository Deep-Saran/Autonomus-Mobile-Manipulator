#!/usr/bin/env python
#This line is very important


"""
Intro goes here
"""
import rospy
import roslib
import tf
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2,sqrt,pow


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

    b=[0,0,2,2,0]
    a=[0,2,2,0,0]
    n=len(a)
    i=0
    for i in xrange(n):

    	p= b[i]
    	q= a[i]
    
    	    
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
			speed.angular.z=1*(angle_to_goal - theta)
			pub.publish(speed)          

    		else:
			speed.linear.x = 0.8*euclidean_distance
			speed.angular.z=0
			pub.publish(speed)
        		if(round(euclidean_distance)==0):
				speed.linear.x = 0
				speed.angular.z=0   
        		        pub.publish(speed)
				break
   
    #rospy.spin()

if __name__=="__main__":
    main()
