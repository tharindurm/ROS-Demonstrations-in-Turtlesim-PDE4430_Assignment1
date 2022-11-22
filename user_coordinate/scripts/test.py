#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import turtlesim.srv
from math import pow,atan2,sqrt,degrees

velocity_publisher = rospy.Publisher('/cleaner/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()

currentPose = Pose()

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
    return val


def normalizedDegrees(degree):
    if degree==0:
        return 0.1
    if isBetween(0,degree,180):
        return degree
    if isBetween(-180,degree,0):
        return 180+(180-abs(degree))



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
    currentX = currentPose.x

    displacement = sqrt(pow(xGoal - currentPose.x,2) + pow(yGoal - currentPose.y,2))
    print("currentX: ",currentX)
    #steering_angle = atan2(yGoal-currentPose.y, xGoal-currentPose.x) - currentPose.theta

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
    #while not isAround(displacement,0,0.01):
    while True:
        #print()
        angle = currentPose.theta
        deg = normalizedDegrees(degrees(angle))

        
        displacement = sqrt(pow(xGoal - currentPose.x,2) + pow(yGoal - currentPose.y,2))
        #if isBetween(170,deg,190):
        #    displacement = sqrt(pow(xGoal - currentPose.x,2) + pow(yGoal - currentPose.y,2))
        #    print("1st: ",displacement)
        #if isBetween(0,deg,10) or isBetween(350,deg,360):
        #    displacement = sqrt(pow(xGoal - currentPose.x,2) + pow(yGoal - currentPose.y,2))
        #    print("2nd: ",displacement, "// currentPose.x: ",currentPose.x)
        
        print("CurrentPose.x: ",currentPose.x)

        #vel_msg.angular.z = 180-deg
        #Angular vel when facing right
        if isBetween(179.95,deg,180.05):
            vel_msg.angular.z = 0.0
        if isBetween(120,deg,180):
            vel_msg.angular.z = 180-deg
            velocity_publisher.publish(vel_msg)
        elif isBetween(181,deg,240):
            vel_msg.angular.z = 180-deg
            velocity_publisher.publish(vel_msg)

        #Angular vel when facing left
        if isBetween(0,deg,60):
            vel_msg.angular.z = 0-deg
            velocity_publisher.publish(vel_msg)
        elif isBetween(300,deg,360):
            vel_msg.angular.z = 360-deg
            velocity_publisher.publish(vel_msg)


        vel_msg.linear.x = clampVal(displacement,0,1)

        print("AngularVel: ",round(vel_msg.angular.z,4)," / angle: ",round(angle,4)," / deg: ",round(deg,4), "Displacement: ",round(displacement,4)," / x_vel: ",vel_msg.linear.x)

        vel_msg.angular.z = 0.0
        velocity_publisher.publish(vel_msg)

        if currentPose.x<0.49 or currentPose.x>10.6:
            print("breaking")
            break

        #if isAround(displacement,0,0.01):
        #    print("breaking")
        #    break

        '''
        if(isBetween(-1.0,angle,1.0)):
            displacement = 10.5-currentPose.x
        elif(isBetween(2.0,angle,3.14) or isBetween(-3.14,angle,-2.0)):
            displacement = abs(0.5-currentPose.x)
        print("Displacement : ",displacement)
        '''
        #displacement = sqrt(pow(xGoal - currentPose.x,2) + pow(yGoal - currentPose.y,2))


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
    print("Pose updated. x: ",currentPose.x)

def autoMove():
    rospy.init_node('cleaner', anonymous=True)
    pose_subscriber = rospy.Subscriber('/cleaner/pose',Pose, updateGlobalCurrentPose)
    rate = rospy.Rate(10)
    '''
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
    '''
    
    
    for y in range(1,11):
        print("-=-=-=-=-=-=  Y =",y,"  -=-=-=-=-=-=")
        moveLong(10.5,y-0.5)
        print("Moved to right")
        rotate(0.5,90,0)
        print("right side rotate done")
        move(0.4,0.5)
        print("right side moved up")
        rotate(0.5,90,0)
        print("right side goback rotate")
        moveLong(0.5,y)
        print("Moved to left")
        rotate(0.5,90,1)
        print("left side rotate done")
        move(0.4,0.5)
        print("left side moved up")
        rotate(0.5,90,1)
        print("left side go back rotate")
        print("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=")
    
    
    '''
    moveLong(10.5,0.5)
    rotate(0.5,90,0)
    move(0.4,0.5)
    rotate(0.5,90,0)
    moveLong(0.5,1)
    print("Reached x=0.5-=====-=-=-=-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-==--")
    rotate(0.5,90,1)
    move(0.4,0.5)
    rotate(0.5,90,1)
    '''
    
    
    
    
    rate.sleep()
    


    rospy.spin()

    
if __name__ == '__main__':
    try:
        autoMove()
    except rospy.ROSInterruptException:
        pass
