#!/usr/bin/env python3
'''

Source referred:

https://github.com/clebercoutof/turtlesim_cleaner/blob/master/src/gotogoal.py

'''

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt


#current_pose = Pose()
goal_pose = Pose()
vel = Twist()

vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)

def inThreshRange(vals,thresh):
    val = abs(vals)
    if (val-thresh)<val and val<(val+thresh):
        return True
    else:
        return False

def clamp_vel(speed, max, min):
    if speed>max:
        return max
    else:
        return min
    return speed

def turnUntil(currentTheta, goalTheta):
    global vel_pub
    turnCmd = Twist()
    if goalTheta<0:
        while(currentTheta>=goalTheta):
            turnCmd.angular.z = -0.2
            vel_pub.publish(turnCmd)
    else:
        while(currentTheta<=goalTheta):
            turnCmd.angular.z = 0.2
            vel_pub.publish(turnCmd)


def moveCommands(currentPoseData):
    
    global goal_pose
    global vel_pub

    displacement = sqrt(pow(goal_pose.x - currentPoseData.x,2) + pow(goal_pose.y - currentPoseData.y,2))
    steering_angle = atan2(goal_pose.y-currentPoseData.y, goal_pose.x-currentPoseData.x) - currentPoseData.theta

    print("current X:",currentPoseData.x," current Y:",currentPoseData.y)
    print("Goal atan2         : ",atan2(goal_pose.y, goal_pose.x))
    print("Current atan2      : ",atan2(currentPoseData.y, currentPoseData.x))
    print("goal-current atan2 : ",atan2(goal_pose.y-currentPoseData.y, goal_pose.x-currentPoseData.x))
    print("current theta      : ",currentPoseData.theta)
    print("calculated atan2   : ",atan2(goal_pose.y-currentPoseData.y, goal_pose.x-currentPoseData.x) - currentPoseData.theta)
    print()


    if displacement>0.2:
        vel.angular.z = atan2(goal_pose.y-currentPoseData.y, goal_pose.x-currentPoseData.x) - currentPoseData.theta
        vel.linear.x = clamp_vel(displacement,1,0.4)
        print("Displacement : ",displacement,"\n")
        vel_pub.publish(vel)
    else:
        vel.angular.z = 0
        vel.linear.x = 0
        print("Displacement : Stopped")
        vel_pub.publish(vel)
        


def autoMove():
    rospy.init_node('turtlebot_controller', anonymous=True)

    rate = rospy.Rate(10)
    goal_pose.x = float(input("Enter destination X coordinate: "))
    goal_pose.y = float(input("Enter destination Y coordinate: "))
    #vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)

    pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, moveCommands)
        
    rospy.spin()


if __name__ == '__main__':
    try:

        autoMove()
            
    except rospy.ROSInterruptException:
        pass