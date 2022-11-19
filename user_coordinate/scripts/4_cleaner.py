#!/usr/bin/env python3
import rospy
import turtlesim.srv
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

currentPose = Pose()

move = Twist()
move.linear.x = 0.0
move.angular.z = 0.0

'''
#Deleting default 'turtle1' instance form turtlesim
rospy.wait_for_service('kill')
killer = rospy.ServiceProxy('kill',turtlesim.srv.Kill)
killer("turtle1")

#Spawning new turtle at a corner of the turtle sim
rospy.wait_for_service('spawn')
spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
spawner(0.5, 0.5, 0, "cleaner")
'''

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


def cleaning(vel_pub):
    x_min,x_max = 0.5,10.5
    y_min,y_max = 0.5,10.5

    global move
    global currentPose

    x = currentPose.x
    y = currentPose.y

    orientation = normalizedDegrees(math.degrees(currentPose.theta))

    if x >= x_min and x <= x_max:
        move.linear.x = 1.0
    else:
        print("X out of range")

    #Turning 90 degrees anticlock wise
    if x > x_max:
        newAngleHeading = 90
        move.linear.x = 0.0
        while not isAround(orientation,newAngleHeading,1):
            print("turning")
            move.angular.z =  0.4
            vel_pub.publish(move)
            orientation = normalizedDegrees(math.degrees(currentPose.theta))
        move.angular.z = 0.0
        vel_pub.publish(move)

    #Check if turtle oriented to up. if so add linear velocity to move up
    if isAround(orientation,90,2):
        moveStraight(0.5,0.8,vel_pub)




    print("Cleaning in progress")
    #vel_pub.publish(move)

def moveStraight(distance,speed,vel_pub):
    global move

    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    while(current_distance < distance):
        t1=rospy.Time.now().to_sec()
        current_distance= speed*(t1-t0)
        print("currentDistance : ",current_distance, "   Distance : ",distance)
        vel_pub.publish(move)

    move.linear.x = 0
    vel_pub.publish(move)


def updateGlobalCurrentPose(data):
    global currentPose
    currentPose = data

def autoClean():
    global currentPose
    global move

    rospy.init_node('cleaner', anonymous=True)
    pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, updateGlobalCurrentPose)
    vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        cleaning(vel_pub)
        rate.sleep()
    
    rospy.spin()


if __name__ == '__main__':
    try:
        autoClean()
    except rospy.ROSInterruptException:
        pass


'''
might be needed in launch file

<node pkg="com760_week5_ws_pkg" type="turtle_tf2_listener.py" name="listener1" output="screen">
    <param name="turtle_number" value="turtle_1" />
    <param name="start_position" value=[0,1,8]>
</node>

'''