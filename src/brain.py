#!/usr/bin/env python
import serial
import rospy
import time
import math
import random
import actionlib
from time import sleep
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

########################## Methods ############################ 

# Interrupt event for commanding the brain
def ExecuteCommand(msg):
    commandValue = int(msg.data)
    if commandValue == 1:
	SetRandomGoal()
    if commandValue == 2:
	CancelGoals()

# Picks a random location out of the apriltag dictionary
def GetRandomLocation():
    index = random.randint(0,len(tagLocations)-1)
    return tagLocations.values()[index]

# Create a MoveBaseGoal with a random location
def GetRandomGoal():
    location = GetRandomLocation()
    goal = MoveBaseActionGoal()

    goal.header.seq = 0
    goal.header.frame_id = "map" 
    goal.header.stamp.secs = rospy.get_rostime().secs 
    goal.header.stamp.nsecs = rospy.get_rostime().nsecs
    goal.goal_id.stamp = rospy.get_rostime()
    goal.goal_id.id = "goal" 
    goal.goal.target_pose.header.seq = 0
    goal.goal.target_pose.header.stamp.secs = rospy.get_rostime().secs 
    goal.goal.target_pose.header.stamp.nsecs = rospy.get_rostime().nsecs
    goal.goal.target_pose.header.frame_id = "map"
    goal.goal.target_pose.pose.position.x = location[0]
    goal.goal.target_pose.pose.position.y = location[1] 
    goal.goal.target_pose.pose.position.z = 0.0
    goal.goal.target_pose.pose.orientation.w = 1.0
	
    return goal

# Publish a random goal
def SetRandomGoal():
    SetGoal(GetRandomGoal())


# Publish a goal on the move_base/goal topic
def SetGoal(goal):
    goalTopic.publish(goal)
    print("Goal set")

# Publish a cancel event to the move_base/cancel
def CancelGoals():
    cancelTopic.publish(GoalID())
    print("Goals canceled")

    

# Publish a random goal as action with feedback from the navigation server
def SetRandomGoalAction():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    client.wait_for_server()
    client.send_goal(GetRandomGoal())
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


###############################################################


# Init ROS components
rospy.init_node('topic_publisher')
commandTopic = rospy.Subscriber("brain_command", Int32, ExecuteCommand);
goalTopic = rospy.Publisher("move_base/goal", MoveBaseActionGoal, queue_size=25)
cancelTopic = rospy.Publisher("move_base/cancel", GoalID, queue_size=25)

# Init global components
commandValue = 0
aprilTag = tuple()

# Build tag location dictionary
tagLocations = {
1:(13.489720850756148, 60.59727947971955, 0.0),
2:(15.934169265195797, 60.933202561559746, 0.0),
3:(20.810319786490506, 60.41751882276302, 0.0),
4:(24.50055443446291, 60.43755116603776, 0.0),
5:(28.07578804452108, 60.138835332979674, 0.0),
6:(31.72803700360133, 60.21433696281538, 0.0),
7:(35.079547458633726, 60.144928942058925, 0.0),
8:(38.86555094259316, 60.061022359125516, 0.0),
9:(42.76742784387715, 60.18446581588793, 0.0),
10:(46.17944236172726, 59.43959194286109, 0.0),
11:(44.86658591585901, 54.932326361019605, 0.0),
12:(41.56825528354129, 55.081602467414015, 0.0),
13:(37.76570212962082, 55.165675939740616, 0.0),
14:(34.22908972386625, 55.347169977516856, 0.0),
15:(30.58898019154235, 55.07027803761846, 0.0),
16:(26.932984917133314, 55.11965596266801, 0.0),
17:(23.205978661221213, 55.18059190272544, 0.0),
18:(19.743041561789195, 55.347504401255364, 0.0),
19:(16.299067248414662, 55.46434990288788, 0.0),
20:(12.100164063844309, 55.42996574075256, 0.0),
21:(9.65735918277374, 55.445542959329764, 0.0),
24:(10.65735918277374, 55.445542959329764, 0.0),
528:(43.65193339638883, 60.123809610098775, 0.0)
}

# Heartbeat for this wonderfull brain
while not rospy.is_shutdown():
	print(commandValue)
	sleep(0.5)

