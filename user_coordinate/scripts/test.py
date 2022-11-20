#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt

velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()

currentPose = Pose()

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
    return val


def rotate(speed,angle,clockwise):
    global vel_msg

    # Receiveing the user's input
    #speed = input("Input your speed (degrees/sec):")
    #angle = input("Type your distance (degrees):")
    #clockwise = input("Clockwise?: ") #True or false

    #Converting from angles to radians
    angular_speed = speed
    relative_angle = angle*2*3.141/360

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        #print("rotating")
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)


def moveLong(xGoal,yGoal):
    global vel_msg
    global velocity_publisher
    global currentPose
    
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
        #print()
        angle = currentPose.theta
        #print(angle)
        if angle<=1.571:
            vel_msg.angular.z = 0-angle
        else:
            vel_msg.angular.z = 3.14-angle
        #print("Angular: ",vel_msg.angular.z)

        vel_msg.linear.x = clampVal(displacement,0,2)
        print("Moving Displacement:",round(displacement,2))
        velocity_publisher.publish(vel_msg)
        displacement = sqrt(pow(xGoal - currentPose.x,2) + pow(yGoal - currentPose.y,2))


def move(speed,distance):
    
    vel_msg.linear.x = speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    while(current_distance < distance):
        velocity_publisher.publish(vel_msg)
        t1=rospy.Time.now().to_sec()
        current_distance= speed*(t1-t0)
        #print(current_distance,"/",distance)
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)


def updateGlobalCurrentPose(data):
    global currentPose
    currentPose = data


def autoMove():
    rospy.init_node('cleaner', anonymous=True)
    pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, updateGlobalCurrentPose)
    rate = rospy.Rate(10)
    
    #while not rospy.is_shutdown():
    moveLong(10,5.5)
    rotate(0.1,90,0)
    move(0.4,0.5)
    rotate(0.1,90,0)
    moveLong(1,6)
    print("Reached x=0.5-=====-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-==--")
    rotate(0.4,90,1)
    move(0.4,0.5)
    rotate(0.4,90,1)
    rate.sleep()
    
    rospy.spin()


if __name__ == '__main__':
    try:
        autoMove()
    except rospy.ROSInterruptException:
        pass
