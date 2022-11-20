#!/usr/bin/env python3
import rospy
import turtlesim.srv
from math import sqrt,atan2,pow, degrees
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

currentPose = Pose()

move = Twist()
move.linear.x = 0.0
move.angular.z = 0.0

#vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
vel_pub = rospy.Publisher('/cleaner/cmd_vel',Twist, queue_size=10)

turn90CW = False
turn90CCW = False

distance = 0

#Deleting default 'turtle1' instance form turtlesim
rospy.wait_for_service('kill')
killer = rospy.ServiceProxy('kill',turtlesim.srv.Kill)
killer("turtle1")

#Spawning new turtle at a corner of the turtle sim
rospy.wait_for_service('spawn')
spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
spawner(0.5, 0.5, 0, "cleaner")


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

def clampVal(val,min,max):
    if val>max:
        return max
    elif val<min:
        return min
    else:
        return val

def moveStraight(xGoal,yGoal):
    global move
    global vel_pub
    
    displacement = sqrt(pow(xGoal - currentPose.x,2) + pow(yGoal - currentPose.y,2))
    steering_angle = atan2(yGoal-currentPose.y, xGoal-currentPose.x) - currentPose.theta

    #print("Displacement : ",displacement, "Steering Angle : ",steering_angle)

    '''
    #Turn towards goal
    while not isAround(steering_angle,0,0.1):
        move.angular.z = clampVal(steering_angle,-1,1) #clamps angular speed between 1 and -1
        move.linear.x = 0.0
        print("Turning Steering angle:",round(steering_angle,2), " angular.z campled:",round(move.angular.z,2))
        vel_pub.publish(move)
        steering_angle = atan2(yGoal-currentPose.y, xGoal-currentPose.x) - currentPose.theta
    '''
    #Move towards goal
    while not isAround(displacement,0,0.1):
        print("Moving")
        move.angular.z = 4*atan2(yGoal-currentPose.y, xGoal-currentPose.x) - currentPose.theta
        move.linear.x = clampVal(displacement,0,1)
        print("Moving Displacement:",round(displacement,2))
        vel_pub.publish(move)
        displacement = sqrt(pow(xGoal - currentPose.x,2) + pow(yGoal - currentPose.y,2))
    
    
    
    
    '''
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    #while(current_distance < distance):
    while not isAround(current_distance,distance,0.1):
        t1=rospy.Time.now().to_sec()
        current_distance= speed*(t1-t0)
        print("currentDistance : ",current_distance, "   Distance : ",distance)
        move.linear.x = speed
        vel_pub.publish(move)
    
    distance = 0
    move.linear.x = 0
    vel_pub.publish(move)
    '''

def cleaning(vel_pub):
    x_min,x_max = 0.5,10.5
    y_min,y_max = 0.5,10.5

    global move
    global currentPose

    x = currentPose.x
    y = currentPose.y

    orientation = normalizedDegrees(degrees(currentPose.theta))

    if x >= x_min and x <= x_max:
        move.linear.x = 1.0
    else:
        print("X out of range")

    #Turning 90 degrees anticlock wise
    if turn90CW:
        newAngleHeading = 90
        move.linear.x = 0.0
        while not isAround(orientation,newAngleHeading,1):
            print("turning")
            move.angular.z =  0.4
            vel_pub.publish(move)
            orientation = normalizedDegrees(degrees(currentPose.theta))
        move.angular.z = 0.0
        vel_pub.publish(move)

    #Check if turtle oriented to up. if so add linear velocity to move up
    if isAround(orientation,90,2):
        moveStraight(0.5,0.8,vel_pub)

    #Turning 90 degrees anticlock wise
    if turn90CCW:
        newAngleHeading = 90
        move.linear.x = 0.0
        while not isAround(orientation,newAngleHeading,1):
            print("turning")
            move.angular.z =  0.4
            vel_pub.publish(move)
            orientation = normalizedDegrees(degrees(currentPose.theta))
        move.angular.z = 0.0
        move.linear.x = 0.8
        vel_pub.publish(move)

    print("Cleaning in progress")
    #vel_pub.publish(move)




def updateGlobalCurrentPose(data):
    global currentPose
    currentPose = data

def autoClean():
    global currentPose
    global move

    rospy.init_node('cleaner', anonymous=True)
    #pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, updateGlobalCurrentPose)
    pose_subscriber = rospy.Subscriber('/cleaner/pose',Pose, updateGlobalCurrentPose)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        #cleaning(vel_pub)
        moveStraight(10.5,0.5)
        moveStraight(10.5,1.5)
        moveStraight(0.5,1.5)
        moveStraight(0.5,2.5)
        moveStraight(10.5,2.5)
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