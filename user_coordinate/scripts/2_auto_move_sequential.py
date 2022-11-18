#!/usr/bin/env python3
'''

Source referred:

https://github.com/clebercoutof/turtlesim_cleaner/blob/master/src/gotogoal.py

'''

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt


current_pose = Pose()
goal_pose = Pose()
vel = Twist()
displacement = 0.0

#Variable to determin if the turtle is in the goal or not. initially its not in the goal hense its True
away = True

vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)

def inThreshRange(vals,thresh):
    val = abs(vals)
    if (val-thresh)<val and val<(val+thresh):
        return True
    else:
        return False

def notInThreshAngle(current, goal, thresh):
    if current<0:
        if current < goal-thresh or current > goal+thresh:
            return True
        else:
            return False
    else:
        if current < goal - thresh or current > goal + thresh:
            return True
        else: 
            return False


def clamp_vel(speed, max, min):
    if speed>max:
        return max
    else:
        return min
    return speed

def turnToGoal():
    print("                 TURNING ------------------------")
    global current_pose
    global goal_pose

    degrees = math.degrees(atan2(goal_pose.y-current_pose.y, goal_pose.x-current_pose.x) - current_pose.theta)

    while notInThreshAngle(degrees,0,10):
        if degrees < 0:
            vel.angular.z = -0.8
        else:
            vel.angular.z = 0.8

        vel_pub.publish(vel)
        degrees = math.degrees(atan2(goal_pose.y-current_pose.y, goal_pose.x-current_pose.x) - current_pose.theta)

def moveCalcualtions(currentPoseData):
    
    global goal_pose
    global vel_pub
    global current_pose
    global displacement
    current_pose = currentPoseData

    displacement = sqrt(pow(goal_pose.x - currentPoseData.x,2) + pow(goal_pose.y - currentPoseData.y,2))
    steering_angle = atan2(goal_pose.y-currentPoseData.y, goal_pose.x-currentPoseData.x) - currentPoseData.theta

    #print("current X:",currentPoseData.x," current Y:",currentPoseData.y)
    #print("Goal atan2         : ",atan2(goal_pose.y, goal_pose.x))
    #print("Current atan2      : ",atan2(currentPoseData.y, currentPoseData.x))
    #print("goal-current atan2 : ",atan2(goal_pose.y-currentPoseData.y, goal_pose.x-currentPoseData.x))
    #print("Current orientation : ",math.degrees(atan2(goal_pose.y-currentPoseData.y, goal_pose.x-currentPoseData.x)))
    #print("current theta      : ",currentPoseData.theta)
    #print("calculated atan2   : ",atan2(goal_pose.y-currentPoseData.y, goal_pose.x-currentPoseData.x) - currentPoseData.theta)
    #print("Current orientation : ",math.degrees(atan2(goal_pose.y-currentPoseData.y, goal_pose.x-currentPoseData.x) - currentPoseData.theta))
    #print()

def move():
    global current_pose
    global displacement
    global away

    print("Move command Started")
    while True:
        if away:
            print("Move command While Loop")
            if displacement>0.2:
                vel.angular.z = atan2(goal_pose.y-current_pose.y, goal_pose.x-current_pose.x) - current_pose.theta
                vel.linear.x = clamp_vel(displacement,1,0.4)
                #print("Displacement : ",displacement,"\n")
                vel_pub.publish(vel)
            else:
                vel.angular.z = 0
                vel.linear.x = 0
                print("Displacement : Stopped")
                vel_pub.publish(vel)
                away = False
    #print("Move command ended")

def autoMove():
    rospy.init_node('turtlebot_controller', anonymous=True)

    rate = rospy.Rate(10)
    goal_pose.x = float(input("Enter destination X coordinate: "))
    goal_pose.y = float(input("Enter destination Y coordinate: "))
    #vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)

    pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, moveCalcualtions)
    turnToGoal()
    move()
    rospy.spin()


if __name__ == '__main__':
    try:

        autoMove()
            
    except rospy.ROSInterruptException:
        pass