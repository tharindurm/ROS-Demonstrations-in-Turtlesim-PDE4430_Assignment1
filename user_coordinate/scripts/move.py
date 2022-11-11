#!/usr/bin/env python3
import rospy
#import keyboard
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import getch
import sys


vel = Twist()

def pose_callback(pose):
	rospy.loginfo("x= %f:Y= %f:Z=%f\n",pose.x,pose.y,pose.theta)
	
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
	rospy.Subscriber('/turtlel1/pose',Pose, pose_callback)
	rate = rospy.Rate(60) 
	lin_vel = 0.5
	ang_vel = 0.2
	rospy.loginfo("Initial values set to lin_vel = %.2f, ang_vel = %.2f",lin_vel,ang_vel)
	while not rospy.is_shutdown():

		c = getch.getch()
		#rospy.loginfo("Pressed = %s",c)

		if c=="q":
			lin_vel +=0.5
		if c=="z":
			lin_vel -=0.5
		
		if c=="e":
			ang_vel +=0.2
		if c=="c":
			ang_vel -=0.2

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
		if c=="x":
			set_twist(0.0,0.0)
			pub.publish(vel)
		
		'''
		if keyboard.is_pressed("a"):
			lin_vel += 0.5
		if keyboard.is_pressed("z"):
			lin_vel -= 0.5
		
		if keyboard.is_pressed("up"):
			lin_vel = lin_vel + 0.5
		if keyboard.is_pressed("down"):
			lin_vel = lin_vel - 0.5
		if keyboard.is_pressed("left"):
			ang_vel = ang_vel + 0.5
		if keyboard.is_pressed("right"):
			ang_vel = ang_vel - 0.5
		'''
 
		rospy.loginfo("Lin_Vel= %.2f : Ang_Vel = %.2f", lin_vel,ang_vel) 
		
		rate.sleep() 

if __name__ == '__main__':
	try:
		#move_turtle(float(sys.argv[1]),float(sys.argv[2]))
		move_turtle()
	except rospy.ROSlnterruptException:
		pass 

