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



def moveCommands(currentPoseData,vel_pub):
    
    global goal_pose

    #calculating displacement
    displacement = sqrt(pow(goal_pose.x - currentPoseData.x,2) + pow(goal_pose.y - currentPoseData.y,2))
    steering_angle = atan2(goal_pose.y-currentPoseData.y, goal_pose.x-currentPoseData.x) - currentPoseData.theta

    #rospy.loginfo("X pos: %.2f, Y pos: %.2f ",currentPoseData.x,currentPoseData.y)
    #rospy.loginfo("Disp : %.2f",displacement)
    #rospy.loginfo(steering_angle)

    vel.angular.z = steering_angle *0.5
    vel.linear.x = displacement * 0.2

    vel_pub.publish(vel)
    print("Steering angle: ",round(vel.angular.z,4),end="")
    print("\tDisplacement: ",round(vel.linear.x,4))
    #print("   ")


def autoMove():
    rospy.init_node('turtlebot_controller', anonymous=True)

    rate = rospy.Rate(10)
    goal_pose.x = float(input("Enter destination X coordinate: "))
    goal_pose.y = float(input("Enter destination Y coordinate: "))
    vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)

    pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, moveCommands,vel_pub)
        
    rospy.spin()


if __name__ == '__main__':
    try:

        autoMove()
            
    except rospy.ROSInterruptException:
        pass