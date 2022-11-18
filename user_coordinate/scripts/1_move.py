#!/usr/bin/env python3

'''
Sources referred:

https://automaticaddison.com/implementing-the-ros-turtlesim-project-with-rospy

'''

import rospy
from geometry_msgs.msg import Twist
import getch

#Twist object created as a global variable
vel = Twist()

#This function changes the Twist object content with parameters
def set_twist(linear_v, angular_v):
	global vel
	vel.linear.x = linear_v 
	vel.linear.y = 0 
	vel.linear.z = 0
		 
	vel.angular.x = 0 
	vel.angular.y = 0
	vel.angular.z = angular_v
		
def move_turtle(): 
	rospy.init_node('move_turtle', anonymous=False)
	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10) 
	rate = rospy.Rate(60) 

	#Initializing velocity values
	lin_vel = 1.0
	ang_vel = 0.8

	rospy.loginfo("\n\nInitial values set to lin_vel = %.2f, ang_vel = %.2f\n\n",lin_vel,ang_vel)
	
	while not rospy.is_shutdown():

		c = getch.getch()

		#Letters 'q' and 'z' are being used to change linear velocity
		if c=="q":
			lin_vel +=0.5
		if c=="z":
			lin_vel -=0.5
		
		#Letters 'e' and 'c' are being used to change angular velocity
		if c=="e":
			ang_vel +=0.2
		if c=="c":
			ang_vel -=0.2

		#Movements of the robot is controlled using 'a','s','d','w'
		if c=="w":
			set_twist(lin_vel,0)
			pub.publish(vel)
		if c=="s":
			set_twist(-lin_vel,0)
			pub.publish(vel)
		if c=="a":
			set_twist(0,ang_vel)
			pub.publish(vel)
		if c=="d":
			set_twist(0,-ang_vel)
			pub.publish(vel)

		#Letter 'x' stops all the movements of the robot
		if c=="x":
			set_twist(0.0,0.0)
			pub.publish(vel)

		rate.sleep() 


if __name__ == '__main__':
	try:
		move_turtle()
	except rospy.ROSlnterruptException:
		pass 