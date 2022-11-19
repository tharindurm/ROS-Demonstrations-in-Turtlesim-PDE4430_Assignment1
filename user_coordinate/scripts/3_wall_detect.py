#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

currentPose = Pose()

padding = 1

move = Twist()
move.linear.x = 0.0
move.angular.z = 0.0

#vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)

def isBetween(min,num,max):
    if num>=min and num<=max:
        return True
    return False

def isAround(val,dest,thresh):
    if val>dest-thresh and val<dest+thresh:
        return True
    return False

def normalizedDegrees(degree):
    if degree==0:
        return 0.1
    if isBetween(0,degree,180):
        return degree
    if isBetween(-180,degree,0):
        return 180+(180-abs(degree))


def crashAvoidCmd(vel_pub):
    global currentPose
    global move
    global padding

    x = currentPose.x
    y = currentPose.y

    print("Orientation : ",math.degrees(currentPose.theta))
    
    deg = normalizedDegrees(math.degrees(currentPose.theta))


    if y > 11 - padding:
        print("Edge detected")
        
        if isBetween(90,deg,180):
            newAngleHeading = 180+(180-deg)
            
            while not isAround(deg,newAngleHeading,5):
                move.angular.z =  4 * (11 - y)
                vel_pub.publish(move)
                deg = normalizedDegrees(math.degrees(currentPose.theta))
            move.angular.z = 0.0
            vel_pub.publish(move)
        

        if isBetween(0,deg,90):
            newAngleHeading = normalizedDegrees(0-deg)
            
            while not isAround(deg,newAngleHeading,2.5):
                move.angular.z =  -4 * (11 - y)
                vel_pub.publish(move)
                deg = normalizedDegrees(math.degrees(currentPose.theta))
            
            move.angular.z = 0.0
            vel_pub.publish(move)

        print("Phew....Turned back")

    if y < 0 + padding:
        print("Edge detected")
        if isBetween(270,deg,360):
            newAngleHeading = normalizedDegrees(360-deg)
            
            while not isAround(deg,newAngleHeading,5):
                move.angular.z =  4 * abs((0-y))
                vel_pub.publish(move)
                deg = normalizedDegrees(math.degrees(currentPose.theta))
            move.angular.z = 0.0
            vel_pub.publish(move)
        

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



    if x < 0 + padding:
        print("Edge detected")
        if isBetween(90,deg,180):
            newAngleHeading = 90-(90-(180-deg))
            
            while not isAround(deg,newAngleHeading,2.5):
                move.angular.z =  -4 * (abs(0 - x))
                vel_pub.publish(move)
                deg = normalizedDegrees(math.degrees(currentPose.theta))
            
            move.angular.z = 0.0
            vel_pub.publish(move)

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
    
    if x > 11 - padding:
        print("Edge detected")
        if isBetween(0,deg,90):
            newAngleHeading = 90+(90-deg)
            
            while not isAround(deg,newAngleHeading,2.5):
                move.angular.z =  4 * (11-x)
                vel_pub.publish(move)
                deg = normalizedDegrees(math.degrees(currentPose.theta))
            
            move.angular.z = 0.0
            vel_pub.publish(move)

        if isBetween(270,deg,360):
            newAngleHeading = 180+(360-deg)

            while not isAround(deg,newAngleHeading,2.5):
                move.angular.z =  -4 * (11-x)
                vel_pub.publish(move)
                deg = normalizedDegrees(math.degrees(currentPose.theta))
            
            move.angular.z = 0.0
            vel_pub.publish(move)
        print("Phew....Turned back")
    

    
   

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