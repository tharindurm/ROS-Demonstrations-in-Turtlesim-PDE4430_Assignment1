#!/usr/bin/env python3

#Student ID: M00909166

'''
Source referred:
https://automaticaddison.com/implementing-the-ros-turtlesim-project-with-rospy
https://github.com/clebercoutof/turtlesim_cleaner/blob/master/src/gotogoal.py
'''


import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

#Global variable to store the current pose information of the turtle
current_pose = Pose()

#Global variable to store goal coordinate information
goal_pose = Pose()

#Global variable to store velocity command messages
vel = Twist()

#Stores the displacement between the turtle and the goal
displacement = 0.0

#Threshold value to make value comparisons true
thresh = 0.2

#Variable to determin if the turtle is in the goal or not. initially its not in the goal hense its True
away = True

vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)

#Checks if a value is in the thershold range from the goal value
def notInThreshAngle(current, goal, thresh):
    if current < goal-thresh or current > goal+thresh:
        return True
    else:
        return False
    
#Clamps the return value between minimum and maximum values
def clamp(speed, max, min):
    if speed>=max:
        return max
    elif speed<=min:
        return min
    return speed


#Orient the turtle towards the goal coordinate before start moving.
#This reduces the complications  that could arise when calculating the angular velocity while moving forward
def turnToGoal():
    print("Turning towards GOAL")
    global current_pose
    global goal_pose

    #Calculate the angle between the turtle and goal
    degrees = math.degrees(atan2(goal_pose.y-current_pose.y, goal_pose.x-current_pose.x) - current_pose.theta)

    while notInThreshAngle(degrees,0,10):
        if degrees < 0:
            vel.angular.z = -0.8
        else:
            vel.angular.z = 0.8

        vel_pub.publish(vel)
        #Same as previous but this line pdates the angle between the robot and the goal
        degrees = math.degrees(atan2(goal_pose.y-current_pose.y, goal_pose.x-current_pose.x) - current_pose.theta)


#Callback function of the subscriber. Continuously update currentPose and displacement variables
def moveCalcualtions(currentPoseData):
    
    global goal_pose
    global current_pose
    global displacement
    current_pose = currentPoseData

    displacement = sqrt(pow(goal_pose.x - currentPoseData.x,2) + pow(goal_pose.y - currentPoseData.y,2))
    
#Publish velocity commands to move the turtle towards the goal
def moveToGoal():
    global current_pose
    global displacement
    global away
    global thresh

    print("Move command Started")
    #This loop will move the turtle towards the goal untill the displacement drops in to threshold range of the goal
    while not displacement<thresh:
        if away:
            print("Moving towards goal")
            if displacement>thresh:
                #Calculating angle difference between goal and turtle. The same value used as angular velocity
                #If turtle moves towards the goal, differance is 0 resulting 0 angular velocity.
                vel.angular.z = atan2(goal_pose.y-current_pose.y, goal_pose.x-current_pose.x) - current_pose.theta

                #Clamping linear velocity to reduce the max speed
                vel.linear.x = clamp(displacement,1,0.4)
                #print("Displacement : ",displacement,"\n")
                vel_pub.publish(vel)
            else:
                #Setting everthing velocity commands to 0 to stop the turtle
                vel.angular.z = 0
                vel.linear.x = 0
                print("Reached detination")
                vel_pub.publish(vel)
                #Turtle is not away from the goal anymore. so make it false
                away = False
    
    print("Task 2 Completed. Exiting now")


def autoMove():
    rospy.init_node('turtlebot_controller', anonymous=True)

    rate = rospy.Rate(10)

    pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, moveCalcualtions)

    #Getting goal coordinated ffrom the user
    goal_pose.x = float(input("Enter destination X coordinate: "))
    goal_pose.y = float(input("Enter destination Y coordinate: "))

    turnToGoal()
    moveToGoal()
    
    

if __name__ == '__main__':
    try:
        autoMove()    
    except rospy.ROSInterruptException:
        pass