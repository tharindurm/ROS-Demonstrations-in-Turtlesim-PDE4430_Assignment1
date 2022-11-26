# PDE4430 Coursework
## Introduction
 
This coursework is focused on completing 5 Tasks using the ROS framework. The tasks are as follow;
 
- Teleoperation using the keyboard, with an option to change movement speed
- Autonomous navigation to any given coordinate in the Turtlesim window
- Avoiding wall collision – Override movement if wall hitting is imminent
- Vacuum cleaning behavior – Covering the entire window in an efficient manner
- Multiple turtles vacuum cleaning behavior – 2 is good, 3 or more is great
 
The nodes to achieve each task were coded using Python and each file name starts with 
the task number for easy identification. For the last task, 4 individual files were created, responsible for controlling 4 robots spawned in the turtlesim.
 
Each file requires the 'Turtlesim' node to be running to see the output. 
To make the execution of the programs easy separate launch files were created 
and are available in the 'launch' folder 
 
Use the following command with the relevant launch file name to execute the nodes
 
Command:
```bash
  roslaunch <Package_Name> <Launch_file>.launch
```
 
 
Eg:
```bash
  roslaunch user_coordinates task_1.launch
```
 
## Task 1: Teleoperation for Turtlesim
### Node behavior
This node is capable of capturing the keyboard inputs from 
the user to control the movements of the turtle in the turtlesim. 
 
### Node explanation
The node gives the user 2 options to choose from; 
 
**Option 1:** Allows the user to use separate keys to move the turtle and adjust linear and angular velocities of the turtle
 
**Option 2:** Allows the user to change the velocity values by tapping the same key and the turtle will keep moving
according to the adjusted velocity values.
 
The program will print the assigned buttons for each activity at the beginning.
 
![App Screenshot](https://via.placeholder.com/468x300?text=App+Screenshot+Here)
 
The `getch` library is used to capture the keyboard inputs from the user and decide the values for 
linear and angular velocities. These values are stored in a `Twist` message and then published
into the `cmd_vel` topic to move the turtle.
 
## Task 2: Autonomous navigation to goal
### Node behavior
This ROS node will ask the user to enter the `x` and `y` coordinates of the destination and the 
turtle first turns towards the goal and then moves towards it.
 
### Node explanation
The node subscribes to the `pose` topic of the turtle to know its current position and orientation of it in the
turtlesim. The displacement between the goal and the robot is calculated using 'Pythagoras's theorem'
 
```python
 displacement = sqrt(pow(goal_pose.x - currentPoseData.x,2) + pow(goal_pose.y - currentPoseData.y,2))
```
 
The angle robot needs to turn to face the goal is calculated using the below equation which is based on trigonometry.
 
```python
angular.speed = atan2(goal_pose.y-current_pose.y, goal_pose.x-current_pose.x) - current_pose.theta
```
 
These values are used as the linear velocity and angular velocities this makes the turtle move towards
the goal faster when it is far away from the goal and gradually slows down when 
closing in on the destination coordinates. Once it reaches the goal, the node terminates.
By default, the turtle reaches its goal with an accuracy of (+/-)0.2 turtlesim units. This can be 
changed by changing the value of the `thresh` global variable at the beginning of the code
 
![App Screenshot](https://via.placeholder.com/468x300?text=App+Screenshot+Here)
 
 
## Task 3: Wall detection and avoidance
### Node behavior
This node will stop the turtle from bouncing into the walls of the turtlesim. When the 
turtle approach a wall, this node will make an evasive maneuver to avoid the collision.
 
### Node explanation
When the node starts user can either choose to control the turtle manually or make it move
automatically. To implement the bounce back with an angle similar to the angle of attack 
when it is on collision course with the wall, `pose.theta` which represents the orientation of
the turtle in radians is converted into degrees which makes it easier to comprehend during the 
implementation.
 
This node subscribes to the `pose` topic of the turtle and publishes velocity commands to the `cmd_vel` topic
to make the turtle move. The only purpose of the callback function of the subscriber is
to update a global variable `currentPose` which can be accessed by the other function to 
determine the position and the orientation of the turtle.
 
![App Screenshot](https://via.placeholder.com/468x300?text=App+Screenshot+Here)
 
## Task 4: Vacuum cleaning behavior
### Node behavior
This node makes the turtle move in a zigzag path to cover the stage area of the 
turtlesim mimicking a vacuum robot cleaning a room.
 
### Node explanation
The default turtle in the middle of the turtlesim removed via the `kill` service provided
with turtlesim and a new turtle is spawned via the service `spawn` at the bottom left 
corner facing right. 
 
```python
#Deleting default 'turtle1' instance form turtlesim
rospy.wait_for_service('kill')
killer = rospy.ServiceProxy('kill',turtlesim.srv.Kill)
killer("turtle1")
 
#Spawning a new turtle at a corner of the turtlesim named cleaner
rospy.wait_for_service('spawn')
spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
spawner(0.5, 0.5, 0,"cleaner")
```
 
With the new turtle, 3 new topics are created which can be easily identified due to its
naming convention
```bash
$ rostopic list
/cleaner/cmd_vel
/cleaner/color_sensor
/cleaner/pose
/rosout
/rosout_agg
```
 
The turtle will move towards the top edge of the turtlesim following a zigzag path.
To achieve this behavior 4 functions are mainly being used which focus on
moving horizontally, moving vertically, turning left, and turning right. To turn 90 degrees
precisely, the robot will turn slowly to avoid overshooting the limit. The speed of the turtle
is also reduced when it reaches an edge.
 
![App Screenshot](https://via.placeholder.com/468x300?text=App+Screenshot+Here)
 
 
 
 
## Task 5: Multiple turtles vacuum cleaning behavior
### Node behavior
The turtlesim stage is traversed by 4 turtles mimicking the behavior of vacuum-cleaning robots
cleaning a room
 
### Node explanation
The same code used in task 4 is reused here. Instead of using one node to 
control all 4 turtles, 4 individual nodes are created to control each turtle.
In each node, the spawning position of the robot is changed to spawn turtles in 
different places 

