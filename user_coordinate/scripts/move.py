#!/usr/bin/env python3
import rospy
import keyboard
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import sys

def pose_callback(pose):
	rospy.loginfo("x= %f:Y= %f:Z=%f\n",pose.x,pose.y,pose.theta)
	
def move_turtle(): 
	rospy.init_node('move_turtle', anonymous=False)
	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10) 
	rospy.Subscriber('/turtlel1/pose',Pose, pose_callback)
	rate = rospy.Rate(10)
	vel = Twist() 
	lin_vel = 0
	ang_vel = 0
	while not rospy.is_shutdown():
	
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

		vel.linear.x = lin_vel 
		vel.linear.y = 0 
		vel.linear.z = 0
		 
		vel.angular.x = 0 
		vel.angular.y = 0
		vel.angular.z = ang_vel
		 
		rospy.loginfo("Linear Vel = %f: Angular Vel = %f", lin_vel,ang_vel) 
		
		pub.publish(vel)
		
		rate.sleep() 

if __name__ == '__main__':
	try:
		#move_turtle(float(sys.argv[1]),float(sys.argv[2]))
		move_turtle()
	except rospy.ROSlnterruptException:
		pass 

