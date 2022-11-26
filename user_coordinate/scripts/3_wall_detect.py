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

#Global variable to store the current postion details of the turtle
currentPose = Pose()

#This variable decides the width of the danger zone around the edges. smaller the value, closer the turtle will move towards the edge
padding = 1

#Creating command velocity message and initializing velocities to 0
move = Twist()
move.linear.x = 0.0
move.angular.z = 0.0


#Checks if a number is between a given range
def isBetween(min,num,max):
    if num>=min and num<=max:
        return True
    return False

#Checks if a value is closer to a goal value or is around the threshold value 
def isAround(val,dest,thresh):
    if val>dest-thresh and val<dest+thresh:
        return True
    return False

#Converts degrees ranging from (-180)->0->180 to 0->359
def normalizedDegrees(degree):
    if degree==0:
        #This returns 0.1 instead of 0 to avoid conditional checking problems in latter part of the code
        return 0.1
    if isBetween(0,degree,180):
        return degree
    if isBetween(-180,degree,0):
        return 180+(180-abs(degree))


#Detects the edge area while calculating the angle of attack with the danger zone.
#the turtle will bounce back in a angle smilar to its collision angle
def crashAvoidCmd(vel_pub):
    global currentPose
    global move
    global padding

    #Assiging x,y coodinates in to variable with short names for code clarity
    x = currentPose.x
    y = currentPose.y

    #Prints current orientation of the turtle. 'math' library is used to 
    print("Current rientation : ",math.degrees(currentPose.theta))
    
    #Converts orientation of the robot from radians to 0->360 degrees
    deg = normalizedDegrees(math.degrees(currentPose.theta))

    #Checking if the turtle is closer to the TOP
    if y > 11 - padding:
        print("Edge detected")
        
        #Checking colision angle
        if isBetween(90,deg,180):
            newAngleHeading = 180+(180-deg)
            
            while not isAround(deg,newAngleHeading,5):
                move.angular.z =  4 * (11 - y)
                vel_pub.publish(move)
                deg = normalizedDegrees(math.degrees(currentPose.theta))
            move.angular.z = 0.0
            vel_pub.publish(move)
        
        #Checking collision angle 
        if isBetween(0,deg,90):
            newAngleHeading = normalizedDegrees(0-deg)
            
            while not isAround(deg,newAngleHeading,2.5):
                move.angular.z =  -4 * (11 - y)
                vel_pub.publish(move)
                deg = normalizedDegrees(math.degrees(currentPose.theta))
            
            move.angular.z = 0.0
            vel_pub.publish(move)

        print("Phew....Turned back")

    #Checking if the turtle is closer to the BOTTOM
    if y < 0 + padding:
        print("Edge detected")

        #Checking colision angle
        if isBetween(270,deg,360):
            newAngleHeading = normalizedDegrees(360-deg)
            while not isAround(deg,newAngleHeading,5):
                move.angular.z =  4 * abs((0-y))
                vel_pub.publish(move)
                deg = normalizedDegrees(math.degrees(currentPose.theta))
            move.angular.z = 0.0
            vel_pub.publish(move)
        
        #Checking colision angle
        if isBetween(180,deg,270):
            newAngleHeading = normalizedDegrees(180-(deg-180))
            while not isAround(deg,newAngleHeading,2.5):
                move.angular.z =  -4 * abs((0-y))
                print("angular: ",move.angular.z)
                vel_pub.publish(move)
                deg = normalizedDegrees(math.degrees(currentPose.theta))
            
            move.angular.z = 0.0
            vel_pub.publish(move)
        print("Phew....Turned back")


    #Checking if the turtle is closer to the LEFT
    if x < 0 + padding:
        print("Edge detected")

        #Checking colision angle
        if isBetween(90,deg,180):
            newAngleHeading = 90-(90-(180-deg))
            
            while not isAround(deg,newAngleHeading,2.5):
                move.angular.z =  -4 * (abs(0 - x))
                vel_pub.publish(move)
                deg = normalizedDegrees(math.degrees(currentPose.theta))
            
            move.angular.z = 0.0
            vel_pub.publish(move)

        #Checking colision angle
        if isBetween(180,deg,270):
            print("3rd quad")
            newAngleHeading = normalizedDegrees(0-(deg-180))

            while not isAround(deg,newAngleHeading,2.5):
                move.angular.z =  4 * (abs(0-x))
                vel_pub.publish(move)
                deg = normalizedDegrees(math.degrees(currentPose.theta))
            
            move.angular.z = 0.0
            vel_pub.publish(move)
        print("Phew....Turned back")
    

    #Checking if the turtle is closer to the RIGHT       
    if x > 11 - padding:
        print("Edge detected")

        #Checking colision angle
        if isBetween(0,deg,90):
            newAngleHeading = 90+(90-deg)
            
            while not isAround(deg,newAngleHeading,2.5):
                move.angular.z =  4 * (11-x)
                vel_pub.publish(move)
                deg = normalizedDegrees(math.degrees(currentPose.theta))
            
            move.angular.z = 0.0
            vel_pub.publish(move)
        
        #Checking colision angle
        if isBetween(270,deg,360):
            newAngleHeading = 180+(360-deg)

            while not isAround(deg,newAngleHeading,2.5):
                move.angular.z =  -4 * (11-x)
                vel_pub.publish(move)
                deg = normalizedDegrees(math.degrees(currentPose.theta))
            
            move.angular.z = 0.0
            vel_pub.publish(move)
        print("Phew....Turned back")


#Callback function of the subscriber. Continuosly updates the currentPose variable
def updateGlobalCurrentPose(data):
    global currentPose
    currentPose = data
    

def autoMove():
    global currentPose
    global move

    rospy.init_node('edge_avoider', anonymous=True)
    pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, updateGlobalCurrentPose)
    vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    #If auatomatic option is selected the turtle will move forward in a conatsnt speed
    #If not a teleop function needs to be used to move the robot
    aDrive = input("Drive turtle automatically? [Y/N]  : ")
    
    while not rospy.is_shutdown():
        crashAvoidCmd(vel_pub)
        if aDrive=="y" or aDrive=="Y":
            move.linear.x = 0.8
            vel_pub.publish(move)
        rate.sleep()
    
    rospy.spin()


if __name__ == '__main__':
    try:
        autoMove()
    except rospy.ROSInterruptException:
        pass