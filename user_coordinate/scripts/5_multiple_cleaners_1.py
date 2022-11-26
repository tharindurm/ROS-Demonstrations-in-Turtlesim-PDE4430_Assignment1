#!/usr/bin/env python3

#Student ID: M00909166


'''
Referred sources
https://github.com/aniskoubaa/ros_essentials_cpp/blob/master/src/topic02_motion/turtlesim/turtlesim_cleaner.py
'''


import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import turtlesim.srv
from math import radians,degrees
from std_srvs.srv import Empty

#Global variable to store the current postion details of the turtle
currentPose = Pose()
x = currentPose.x
y = currentPose.y

#Creating publisher
vel_pub = rospy.Publisher('/cleaner_1/cmd_vel',Twist, queue_size=10)

#Deleting default 'turtle1' instance form turtlesim
rospy.wait_for_service('kill')
killer = rospy.ServiceProxy('kill',turtlesim.srv.Kill)
killer("turtle1")

#Spawning new turtle at a corner of the turtle sim
rospy.wait_for_service('spawn')
spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
spawner(0.25, 0.250, 0,"cleaner_1")

#Checks if a number is between a given range
def isBetween(val,min,max):
    if val>=min and val<=max:
        return True
    return False

#Used to determine the direction the turtle is facing using the orientation of the turtle
def heading360(deg):
    if isBetween(deg,0,44) or isBetween(deg,316,360):
        return 0
    if isBetween(deg,45,134):
        return 90
    if isBetween(deg,135,224):
        return 180

#Clamps the return value between minimum and maximum values
def clamp(val,min,max):
    if val<min:
        return min
    elif val>max:
        return max
    else:
        return val

#Used to move the turtle in a horizontal axis. distance calculation uses current 'x' position of the turtle
def moveHorizontal(dist):
    global currentPose
    global vel_pub

    x_move = Twist()
    x_move.linear.x = 0.8

    startx = currentPose.x
    totalDistance = 0

    #In the begining of execution due to synchronization problems currentPose variable migh not be updated timely.
    #Having 0 breaks the code. Busy waiting untill the currentPose receives values from the subscriber callback function
    while currentPose.x==0:
        pass

    #The loop will execute untill the distance travelled is less than the specified distance given as the parameter
    while totalDistance<=dist:
        totalDistance = abs(currentPose.x - startx)
        print("Distance travelled : ",totalDistance)
        x_move.linear.x = clamp(dist-totalDistance,0.8,2)
        vel_pub.publish(x_move)

    #Stops the turtle once the specified distance is travelled
    x_move.linear.x = 0.0
    vel_pub.publish(x_move)
    print("Horizontal move ended")

#Used to move the turtle in a vertical axis. distance calculation uses current 'y' position of the turtle
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

    
#Turns the turtle 90 degrees anti-clockwise
def turnLeft90():
    global currentPose
    global vel_pub

    #Setting velocity commands to 0
    turning = Twist()
    turning.linear.x = 0
    turning.angular.z = 0

    #Calculating final orientation of the robot after the turning
    finalAngle = currentPose.theta + radians(90)

    #Determining heading of the turtle an converting it in to radians
    finalAngle = radians(heading360(degrees(finalAngle)))

    # -0.001 is crucial otherwise it takes lots of time to match the angles exactly for many decimal places
    while(currentPose.theta <= finalAngle-0.001):
        print(degrees(currentPose.theta)," --> ",degrees(finalAngle))
        turning.angular.z = abs(currentPose.theta - finalAngle)
        vel_pub.publish(turning)

    #Stopping once the turtle is turned towards the correct orientation
    turning.linear.x = 0
    turning.angular.z = 0
    vel_pub.publish(turning)
    print("Turning Left Done")

#Turns the turtle 90 degrees clockwise
def turnRight90():
    global currentPose
    global vel_pub

    #Setting velocity commands to 0
    turning = Twist()
    turning.linear.x = 0
    turning.angular.z = 0

    #Calculating final orientation of the robot after the turning
    finalAngle = currentPose.theta - radians(90)

    #Determining heading of the turtle an converting it in to radians
    finalAngle = radians(heading360(degrees(finalAngle)))

    # -0.001 is crucial otherwise it takes lots of time to match the angles exactly for 5 decimals
    while(currentPose.theta >= finalAngle+0.001):
        print(degrees(currentPose.theta)," --> ",degrees(finalAngle))
        turning.angular.z = -1 * abs(currentPose.theta - finalAngle)
        vel_pub.publish(turning)

    #Stopping once the turtle is turned towards the correct orientation
    turning.linear.x = 0
    turning.angular.z = 0
    vel_pub.publish(turning)
    print("Turning Right Done")


#Callback function of the subscriber. Continuosly updates the currentPose variable
def updateGlobalCurrentPose(data):
    global currentPose
    currentPose = data

def autoMove():
    global currentPose
    global vel_pub

    rospy.init_node('cleaner_1', anonymous=True)
    pose_subscriber = rospy.Subscriber('/cleaner_1/pose',Pose, updateGlobalCurrentPose)
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



    rospy.spin

if __name__ == '__main__':
    try:
        autoMove()
    except rospy.ROSInterruptException:
        pass