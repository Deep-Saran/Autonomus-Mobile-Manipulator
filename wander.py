#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from math import atan2,sqrt,pow

x0=0
x1=0
x2=0
x3=0
x4=0
x5=0
x6=0
x7=0

y0=0
y1=0
y2=0
y3=0
y4=0
y5=0
y6=0
y7=0
p1=0
p2=0

def callback(data):
    global x0,x1,x2,x3,x4,x5,x6,x7
    global y0,y1,y2,y3,y4,y5,y6,y7

    x0= data.points[0].x
    x1= data.points[1].x
    x2= data.points[2].x
    x3= data.points[3].x
    x4= data.points[4].x
    x5= data.points[5].x
    x6= data.points[6].x
    x7= data.points[7].x

    y0= data.points[0].y
    y1= data.points[1].y
    y2= data.points[2].y
    y3= data.points[3].y
    y4= data.points[4].y
    y5= data.points[5].y
    y6= data.points[6].y
    y7= data.points[7].y

def callback2(data):
    global p1,p2
    p1 = data.pose.pose.position.x
    p2 = data.pose.pose.position.y

def euc(x,y):

    euclidean_distance = sqrt(pow((x - round(p1,6)), 2) + pow((y - round(p2,6)), 2))	
    return 

def main():
    global x0,x1,x2,x3,x4,x5,x6,x7
    global y0,y1,y2,y3,y4,y5,y6,y7
    global p1,p2
    rospy.init_node("mynode", anonymous=True)
    sub=rospy.Subscriber('/sonar',PointCloud,callback)
    sub1=rospy.Subscriber('/odom',Odometry,callback2)
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(100)
    speed = Twist()

    while not rospy.is_shutdown():
	if (euc(x0,y0) < 0.5):
		speed.linear.x= 0
		speed.angular.z=atan2(y0,x0)
		pub.publish(speed)
	elif (euc(x1,y1) < 0.5):
		speed.linear.x= 0
		speed.angular.z=atan2(y1,x1)
		pub.publish(speed)
	elif (euc(x2,y2) < 0.5):
		speed.linear.x= 0
		speed.angular.z=atan2(y2,x2)
		pub.publish(speed)
	elif (euc(x3,y3) < 0.5):
		speed.linear.x= 0
		speed.angular.z=atan2(y3,x3)
		pub.publish(speed)
	elif (euc(x4,y4) < 0.5):
		speed.linear.x= 0
		speed.angular.z=atan2(y4,x4)
		pub.publish(speed)
	elif (euc(x5,y5) < 0.5):
		speed.linear.x= 0
		speed.angular.z=atan2(y5,x5)
		pub.publish(speed)
	elif (euc(x6,y6) < 0.5):
		speed.linear.x= 0
		speed.angular.z=atan2(y6,x6)
		pub.publish(speed)
	elif (euc(x7,y7) < 0.5):
		speed.linear.x= 0
		speed.angular.z=atan2(y7,x7)
		pub.publish(speed)

	speed.linear.x= 0.1
	speed.angular.z=0	       
	pub.publish(speed)

if __name__=="__main__":
    main()
