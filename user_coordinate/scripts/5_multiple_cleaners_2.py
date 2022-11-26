#!/usr/bin/env python3

'''
Referred sources

https://github.com/aniskoubaa/ros_essentials_cpp/blob/master/src/topic02_motion/turtlesim/turtlesim_cleaner.py

'''


import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import turtlesim.srv
from math import atan2,sqrt,radians,degrees
import time
from std_srvs.srv import Empty

currentPose = Pose()
x = currentPose.x
y = currentPose.y

vel_pub = rospy.Publisher('/cleaner_2/cmd_vel',Twist, queue_size=10)

#Deleting default 'turtle1' instance form turtlesim
#rospy.wait_for_service('kill')
#killer = rospy.ServiceProxy('kill',turtlesim.srv.Kill)
#killer("turtle1")

#Spawning new turtle at a corner of the turtle sim
rospy.wait_for_service('spawn')
spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
spawner(0.25, 5.75, 0,"cleaner_2")

def isBetween(val,min,max):
    if val>=min and val<=max:
        return True
    return False

def heading360(deg):
    if isBetween(deg,0,44) or isBetween(deg,316,360):
        return 0
    if isBetween(deg,45,134):
        return 90
    if isBetween(deg,135,224):
        return 180
    return 270

def clamp(val,min,max):
    if val<min:
        return min
    elif val>max:
        return max
    else:
        return val

def moveHorizontal(dist):
    global currentPose
    global vel_pub

    x_move = Twist()
    x_move.linear.x = 0.8

    startx = currentPose.x
    totalDistance = 0

    while totalDistance<=dist:
        totalDistance = abs(currentPose.x - startx)
        print("TotalDistance : ",totalDistance)
        x_move.linear.x = clamp(dist-totalDistance,0.8,2)
        vel_pub.publish(x_move)

    x_move.linear.x = 0.0
    vel_pub.publish(x_move)
    print("Horizontal move ended")



def moveVertical(distance):
    verticalMove = Twist()
    initY = currentPose.y
    destPos = initY + distance

    while currentPose.y <= destPos:
        verticalMove.linear.x = 0.2
        vel_pub.publish(verticalMove)

    verticalMove.linear.x = 0.0
    vel_pub.publish(verticalMove) 
    print("Vertical move done")

    
def turnLeft90():
    global currentPose
    global vel_pub

    turning = Twist()
    turning.linear.x = 0
    turning.angular.z = 0

    finalAngle = currentPose.theta + radians(90)

    finalAngle = radians(heading360(degrees(finalAngle)))

    # -0.001 is crucial otherwise it takes lots of time to match the angles exactly for 5 decimals
    while(currentPose.theta <= finalAngle-0.001):
        print(degrees(currentPose.theta)," --> ",degrees(finalAngle))
        turning.angular.z = abs(currentPose.theta - finalAngle)
        vel_pub.publish(turning)


    turning.linear.x = 0
    turning.angular.z = 0
    vel_pub.publish(turning)
    print("Turning Left Done")


def turnRight90():
    global currentPose
    global vel_pub

    turning = Twist()
    turning.linear.x = 0
    turning.angular.z = 0

    finalAngle = currentPose.theta - radians(90)

    finalAngle = radians(heading360(degrees(finalAngle)))

    # -0.001 is crucial otherwise it takes lots of time to match the angles exactly for 5 decimals
    while(currentPose.theta >= finalAngle+0.001):
        print(degrees(currentPose.theta)," --> ",degrees(finalAngle))
        turning.angular.z = -1 * abs(currentPose.theta - finalAngle)
        vel_pub.publish(turning)


    turning.linear.x = 0
    turning.angular.z = 0
    vel_pub.publish(turning)
    print("Turning Right Done")


def updateGlobalCurrentPose(data):
    global currentPose
    currentPose = data

def autoMove():
    global currentPose
    global vel_pub

    rospy.init_node('cleaner_2', anonymous=True)
    pose_subscriber = rospy.Subscriber('/cleaner_2/pose',Pose, updateGlobalCurrentPose)
    rate = rospy.Rate(10)
    
    moveHorizontal(5)
    for i in range(5):
        turnLeft90()
        moveVertical(0.5)
        turnLeft90()
        moveHorizontal(5)
        turnRight90()
        moveVertical(0.5)
        turnRight90()
        moveHorizontal(5)

if __name__ == '__main__':
    try:
        autoMove()
    except rospy.ROSInterruptException:
        pass