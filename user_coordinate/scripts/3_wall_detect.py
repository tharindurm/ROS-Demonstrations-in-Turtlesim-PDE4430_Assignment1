#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

currentPose = Pose()
move = Twist()
move.linear.x = 0.8
move.angular.z = 0.0

vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)

def isBetween(min,num,max):
    if num>=min and num<=max:
        return True
    return False

def isAround(val,dest,thresh):
    if val>dest-thresh and val<dest+thresh:
        return True
    return False

def normalizedDegrees(degree):
    if isBetween(0,degree,180):
        return degree
    if isBetween(-180,degree,0):
        return 180+(180-abs(degree))


def crashAvoidCmd():
    global currentPose
    global move
    global vel_pub

    x = currentPose.x
    y = currentPose.y

    padding = 1

    print("Degrees : ",math.degrees(currentPose.theta), "Theta : ",currentPose.theta)
    
    deg = normalizedDegrees(math.degrees(currentPose.theta))

    attackAngle = 180 - deg

    if y > 11 - padding:
        if isBetween(90,deg,180):
            newAngleHeading = 180+(180-deg)
            print("180 to 90")
            print("degrees : ", deg)
            print("newAngleHeading : ",newAngleHeading)
            print("attackAngle : ",attackAngle)

            print(isAround(deg,newAngleHeading,5))
            
            while not isAround(deg,newAngleHeading,5):
                print("turning")
                print("Degrees : ",round(deg,2),"New heading : ",round(newAngleHeading,2))
                move.angular.z =  4 * (11 - y)
                vel_pub.publish(move)
                deg = normalizedDegrees(math.degrees(currentPose.theta))
            move.angular.z = 0.0
            vel_pub.publish(move)

        if isBetween(0,deg,90):
            move.angular.z = 1.5

   

def updateGlobalCurrentPose(data):
    global currentPose
    currentPose = data
    

def autoMove():
    global currentPose
    global vel_pub
    rospy.init_node('edge_avoider', anonymous=True)
    pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, updateGlobalCurrentPose)
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        crashAvoidCmd()
        vel_pub.publish(move)
        rate.sleep()
    
    rospy.spin()


if __name__ == '__main__':
    try:
        autoMove()
    except rospy.ROSInterruptException:
        pass