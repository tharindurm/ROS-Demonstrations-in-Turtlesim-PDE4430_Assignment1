#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

currentPose = Pose()
move = Twist()
move.linear.x = 0.0
move.angular.z = 0.4

def boundaryDetection():
    global currentPose
    
    x = currentPose.x
    y = currentPose.y

    if x < 1 or x > 10:
        return True
    if y < 1 or y > 10:
        return True

    return False


def crashAvoidCmd():
    global currentPose

    print("Degrees : ",math.degrees(currentPose.theta), "Theta : ",currentPose.theta)
   

def updateGlobalCurrentPose(data):
    global currentPose
    currentPose = data
    

def autoMove():
    global currentPose
    rospy.init_node('edge_avoider', anonymous=True)
    pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, updateGlobalCurrentPose)
    vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)

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