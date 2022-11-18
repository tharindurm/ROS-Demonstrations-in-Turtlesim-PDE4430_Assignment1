#!/usr/bin/env python3
'''

Source referred:

https://github.com/clebercoutof/turtlesim_cleaner/blob/master/src/gotogoal.py

'''

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

vel = Twist()
padding = 1
avoided = False

vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)

'''
def turn(turnAngle):#angle is given in radians
    vel.linear.x = 0.6
    vel.angular.z = 0.8
    currentAngle = 0.0

    t0 = rospy.Time.now().to_sec()

    while currentAngle<turnAngle:
        print("CurrentAngle : ",currentAngle)
        t1 = rospy.Time.now().to_sec()
        currentAngle = vel.angular.z * (t1-t0)
        vel_pub.publish(vel)

    vel.linear.x = 0
    vel.angular.z = 0
    vel_pub.publish(vel)
    print("--------------------------Terminated --------------------------")
'''


def moveCommands(currentPoseData):
    global avoided

    if(currentPoseData.x<=11-padding and currentPoseData.x>0+padding):
        print("There is space in x")
        avoided = True
    else:
        print("Collided with wall in X. Turning back-------------------------------")
        vel.angular.z = 2.0
        vel.linear.x = 1.0
        vel_pub.publish(vel)
        #if(not avoided):
            #turn(1.57)
            #avoided = true
    
    if(currentPoseData.y<=11-padding and currentPoseData.y>0+padding):
        print("There is space in y")
        avoided = False
            
    else:
        print("Collided with wall in Y. Turning back")
        #if(not avoided):
        #    turn(1.57)
        vel.angular.z = 2.0
        vel.linear.x = 1.0
        vel_pub.publish(vel)

def autoMove():
    rospy.init_node('turtlebot_controller', anonymous=True)

    rate = rospy.Rate(10)
    pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, moveCommands)
        
    rospy.spin()


if __name__ == '__main__':
    try:
        autoMove()
    except rospy.ROSInterruptException:
        pass