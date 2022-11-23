#!/usr/bin/env python3

'''
Referred sources

https://github.com/aniskoubaa/ros_essentials_cpp/blob/master/src/topic02_motion/turtlesim/turtlesim_cleaner.py

'''


import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import turtlesim.srv
from math import atan2,sqrt,radians,degrees
import time
from std_srvs.srv import Empty

move = Twist()

currentPose = Pose()
x = currentPose.x
y = currentPose.y

vel_pub = rospy.Publisher('/cleaner/cmd_vel',Twist, queue_size=10)


#Deleting default 'turtle1' instance form turtlesim
rospy.wait_for_service('kill')
killer = rospy.ServiceProxy('kill',turtlesim.srv.Kill)
killer("turtle1")

#Spawning new turtle at a corner of the turtle sim
rospy.wait_for_service('spawn')
spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
spawner(0.5, 0.5, 0,"cleaner")

#Converts radians in to degrees. converts -180 to 0 to 180 range in to 0 to 360
def radTo360(rad):
    if rad>=0:
        return degrees(rad)
    return abs(degrees(rad))+180


def heading360(deg):
    if deg>=135 and deg<=225:
        return 180
    if deg>=0 and deg<=45:
        return 0
    if deg>=325 and deg<=360:
        return 0



def heading(rad):
    deg = round(degrees(rad),1)
    #print("processing in heading function: ",deg)

    if 180>=deg and deg>=100:
        return 180
    if deg>=-180 and deg<-100:
        return -180

    if deg>=0 and deg<=45:
        return 0
    if deg<0 and deg>=-45:
        return 0

    if deg<100 and deg>90:
        return 90
    if deg>-135 and deg<-45:
        #turtle will not turn towards down. so pass
        pass


def isAtEdge(x):
    if x==0:
        return False
    if x>10.5 or x<0.5:
        print("EDGE : ",x)
        return True
    if x>=0.5 and x<=10.5:
        print("Inside : ",x)
        return False

def inRange():
    pass


def moveToGoal(x_goal,y_goal):
    global currentPose
    global displacement

    displacement = sqrt(pow(x_goal - currentPose.x,2) + pow(y_goal - currentPose.y,2))

    while True:
        displacement = sqrt(pow(x_goal - currentPose.x,2) + pow(y_goal - currentPose.y,2))
        #steering_angle = atan2(y_goal-currentPose.y, x_goal-currentPose.x) - currentPose.theta

        try:
            steering_vel = heading360(radTo360(currentPose.theta)) - degrees(currentPose.theta)
            if heading360(radTo360(currentPose.theta))==180:
                if currentPose.theta<0:
                    steering_vel = abs(steering_vel) * -1
                elif currentPose.theta>0:
                    steering_vel = abs(steering_vel)
                else:
                    steering_vel = 0.0
            elif heading360(radTo360(currentPose.theta))==0:
                if currentPose.theta<0:
                    steering_vel = abs(steering_vel)
                elif currentPose.theta>0:
                    steering_vel = abs(steering_vel) * -1
                else:
                    steering_vel = 0.0
        except:
            steering_vel = 0.0

        #print("steering angle = ",heading(currentPose.theta)," - ",round(degrees(currentPose.theta),1)," steering_val:",steering_vel)
        
        move.linear.x = 1        
        move.angular.z = steering_vel
        vel_pub.publish(move)
        
        if isAtEdge(currentPose.x):
            move.angular.z = 0
            move.linear.x = 0
            vel_pub.publish(move)
            print("Reached destination")
            break

        '''
        if displacement>0.1 or not isAtEdge(currentPose.x):
            move.linear.x = 1        
            move.angular.z = steering_vel
            vel_pub.publish(move)
        else:
            move.angular.z = 0
            move.linear.x = 0
            vel_pub.publish(move)
            print("Reached destination")
            break
        '''

def turnLeft90():
    global currentPose
    global vel_pub

    turning = Twist()
    turning.linear.x = 0
    turning.angular.z = 0

    finalAngle = currentPose.theta + radians(90)

    while(currentPose.theta <=finalAngle-0.01):
        print(degrees(currentPose.theta)," --> ",degrees(finalAngle))
        print("currentDegree: ", currentPose.theta," target: ",finalAngle)
        turning.angular.z = abs(currentPose.theta - finalAngle)
        vel_pub.publish(turning)


    print("Done")



def moveVertical(distance):
    verticalMove = Twist()
    initY = currentPose.y
    destPos = initY + distance

    while currentPose.y <= destPos:
        verticalMove.linear.x = 0.2
        vel_pub.publish(verticalMove)

    verticalMove.linear.x = 0.0
    vel_pub.publish(verticalMove) 
    print("Vertical move done")




def turnRight90():
    global currentPose
    global vel_pub

    turning = Twist()
    turning.linear.x = 0
    turning.angular.z = 0

    finalAngle = currentPose.theta - radians(90)

    while(currentPose.theta < finalAngle-0.01):
        print("currentDegree: ", degrees(currentPose.theta)," target: ",degrees(finalAngle))
        print("currentDegree: ", currentPose.theta," target: ",finalAngle)
        turning.angular.z = -abs(currentPose.theta - finalAngle)
        vel_pub.publish(turning)


    print("Done")

def updateGlobalCurrentPose(data):
    global currentPose
    if not data is None:
        currentPose = data

def autoMove():
    global currentPose
    global move
    global vel_pub

    rospy.init_node('cleaner', anonymous=True)
    pose_subscriber = rospy.Subscriber('/cleaner/pose',Pose, updateGlobalCurrentPose)
    rate = rospy.Rate(10)
    
    #while not rospy.is_shutdown():
    
    moveToGoal(10.5,0.5)
    turnLeft90()
    moveVertical(0.5)
    turnLeft90()
    moveToGoal(0.5,1)

    turnRight90()
    moveToGoal(0.5,1.5)
    turnRight90()
    moveToGoal(10.5,1.5)
        #vel_pub.publish(move)
    #    rate.sleep()
    
    #rospy.spin()


if __name__ == '__main__':
    try:
        autoMove()
    except rospy.ROSInterruptException:
        pass











'''

x = 0
y = 0
yaw = 0


def poseCallback(pose_message):
    global x
    global y, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta

    # print "pose callback"
    # print ('x = {}'.format(pose_message.x)) #new in python 3
    # print ('y = %f' %pose_message.y) #used in python 2
    # print ('yaw = {}'.format(pose_message.theta)) #new in python 3


def move(speed, distance, is_forward):
    # declare a Twist message to send velocity commands
    velocity_message = Twist()
    # get current location
    global x, y
    x0 = x
    y0 = y

    if (is_forward):
        velocity_message.linear.x = abs(speed)
    else:
      	velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0

    # we publish the velocity at 10 Hz (10 times a second)
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(
    cmd_vel_topic, Twist, queue_size=10)

    while True:
        rospy.loginfo("Turtlesim moves forwards")
        velocity_publisher.publish(velocity_message)

        loop_rate.sleep()

              # rospy.Duration(1.0)

        distance_moved = abs(
        0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        print distance_moved
        if not (distance_moved < distance):
                    rospy.loginfo("reached")
                    break

        # finally, stop the robot when the distance is moved
        velocity_message.linear.x = 0
        velocity_publisher.publish(velocity_message)


def rotate(angular_speed_degree, relative_angle_degree, clockwise):

    global yaw
    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    # get current location
    theta0 = yaw
    angular_speed = math.radians(abs(angular_speed_degree))

    if (clockwise):
        velocity_message.angular.z = -abs(angular_speed)
    else:
        velocity_message.angular.z = abs(angular_speed)

    angle_moved = 0.0
    # we publish the velocity at 10 Hz (10 times a second)
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while True:
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()

        if (current_angle_degree > relative_angle_degree):
            rospy.loginfo("reached")
            break

    # finally, stop the robot when the distance is moved
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)


def go_to_goal(x_goal, y_goal):
    global x
    global y, yaw

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    while (True):
        K_linear = 0.5
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        linear_speed = distance * K_linear

        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-yaw)*K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)

        # print velocity_message.linear.x
        # print velocity_message.angular.z
        print 'x=', x, 'y=', y

        if (distance < 0.01):
            break


def setDesiredOrientation(desired_angle_radians):
    relative_angle_radians = desired_angle_radians - yaw
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    print relative_angle_radians
    print desired_angle_radians
    rotate(30, math.degrees(abs(relative_angle_radians)), clockwise)


def gridClean():

    desired_pose = Pose()
    desired_pose.x = 1
    desired_pose.y = 1
    desired_pose.theta = 0

    moveGoal(desired_pose, 0.01)

    setDesiredOrientation(degrees2radians(desired_pose.theta))

    move(2.0, 9.0, True)
    rotate(degrees2radians(20), degrees2radians(90), False)
    move(2.0, 9.0, True)
    rotate(degrees2radians(20), degrees2radians(90), False)
    move(2.0, 1.0, True)
    rotate(degrees2radians(20), degrees2radians(90), False)
    move(2.0, 9.0, True)
    rotate(degrees2radians(30), degrees2radians(90), True)
    move(2.0, 1.0, True)
    rotate(degrees2radians(30), degrees2radians(90), True)
    move(2.0, 9.0, True)
    pass


def spiralClean():
    vel_msg = Twist()
    loop_rate = rospy.Rate(1)
    wk = 4
    rk = 0

    while ((currentTurtlesimPose.x < 10.5) and (currentTurtlesimPose.y < 10.5)):
        rk = rk+1
        vel_msg.linear.x = rk
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = wk
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:

        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        # declare velocity publisher
        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(
            cmd_vel_topic, Twist, queue_size=10)

        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        time.sleep(2)

        #move(1.0, 2.0, False)
        #rotate(30, 90, True)
        go_to_goal(1.0, 1.0)
        # setDesiredOrientation(math.radians(90))

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
'''