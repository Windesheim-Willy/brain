#!/usr/bin/env python
import serial
import rospy
import rosgraph
import rostopic
import time
import math
import random
import actionlib
import roslib
from time import sleep
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Point32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from actionlib_msgs.msg import GoalID, GoalStatusArray
from dynamic_reconfigure.parameter_generator import *
from human_detection.msg import Person



########################## Enums ############################

class Command:
	SetRandomGoal = 0
	CancelGoal = 1
	SlowDown = 2
	SpeedUp = 3

class State:
	Autonomous = 0
	Emergency = 1
	HumanControl = 2
	SocialInteraction = 3
	MoveToTag = 4

class Zone:
	All = 0
	T5Yellow = 1
	T5Bridge = 2
	T5Red = 3

class MoveBaseStatus:
	Pending         = 0   # The goal has yet to be processed by the action server
	Active          = 1   # The goal is currently being processed by the action server
	Preempted       = 2   # The goal received a cancel request after it started executing
                      #   and has since completed its execution (Terminal State)
	Succeeded      = 3   # The goal was achieved successfully by the action server (Terminal State)
	Aborted         = 4   # The goal was aborted during execution by the action server due
                      #    to some failure (Terminal State)
	Rejected        = 5   # The goal was rejected by the action server without being processed,
                      #    because the goal was unattainable or invalid (Terminal State)
	Preempting      = 6   # The goal received a cancel request after it started executing
                      #    and has not yet completed execution
	Recalling       = 7   # The goal received a cancel request before it started executing,
                      #    but the action server has not yet confirmed that the goal is canceled
	Recalled        = 8   # The goal received a cancel request before it started executing
                      #    and was successfully cancelled (Terminal State)
	Lost            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server

########################## Globals ############################
currentState = State.Autonomous
currentZone = Zone.All
movebaseStatus = GoalStatusArray()
lastGoalMsg = MoveBaseActionGoal()
lastMoveBaseMsg = Twist()
lastJoystickMsg = Twist()
lastJoystickMsgUpdate = float(0)
lastHumanDetectionUpdate = float(0)
lastEmergencyMsg = Bool()
slowDown = False
socialInteractionActive = False

########################## Methods ############################ 

def IsInRange(a, b, margin):
    if(a < b+margin and a > b-margin):
        return True
    else:
        return False

# Handler while in State.Autonomous
def HandleStateAutonomous():
	global lastMoveBaseMsg
	global movebaseStatus
	global currentZone

	print("Willy is autonomous driving!")



	if len(movebaseStatus.status_list) <= 0 or movebaseStatus.status_list[0].status == MoveBaseStatus.Succeeded:
		if currentZone == Zone.All:
			SetAutonomousMovementGoal()
		else:
			SetZoneMovementGoal()
	motorTopic.publish(GetSpeed(lastMoveBaseMsg))


# Handler while in State.Emergency
def HandlerStateEmergency():
    print("Willy is in a state of emergency")
    motorTopic.publish(Twist())

# Handler while in State.HumanControl
def HandlerStateHumanControl():
    global lastJoystickMsg

    print("Willy is listening to human controls")
    motorTopic.publish(lastJoystickMsg)

# Handler while in State.SocialInteraction
def HandleSocialInteraction():
    print("Willy is talking to someone")
    motorTopic.publish(Twist())

# Handler while in State.MoveToTag
def HandleMoveToTag():
    global lastMoveBaseMsg
    
    print("Willy is moving to a specific location")

    motorTopic.publish(GetSpeed(lastMoveBaseMsg))

# This function is called when willy transitions from one state to another
def HandleTransition(currentState, newState):
    print("Willy is going from state " + str(currentState) + " to state "+ str(newState))
    

def UpdateState():
	global currentState
	global lastJoystickMsgUpdate
	global lastHumanDetectionUpdate
	global lastEmergencyMsg
	global socialInteractionActive
	global slowDown

	print ("Should willy change state?")
    
    # if human input has been recieved within 5 seconds, a human is trying to take over
	humanTakeover = (time.time() - lastJoystickMsgUpdate) < 5

	# if no human detected within 5 seconds, go back to normal speed
	slowDown = ((time.time() - lastHumanDetectionUpdate)  < 5)

	if lastEmergencyMsg.data == True:
		HandleTransition(currentState, State.Emergency)
		currentState = State.Emergency
		return
	else:
		if currentState == State.Emergency:
			HandleTransition(currentState, State.Autonomous)
			currentState = State.Autonomous
			return
		
	if currentState == State.Autonomous:
		if humanTakeover:
			HandleTransition(currentState, State.HumanControl)
			currentState = State.HumanControl
			return
		if socialInteractionActive:
			HandleTransition(currentState, State.SocialInteraction)
			currentState = State.SocialInteraction
			return
	if currentState == State.HumanControl:
		if not humanTakeover:
			HandleTransition(currentState, State.Autonomous)
			currentState = State.Autonomous
    #TODO check if in range of move goal for tag


# Interuppt for move_base/status updates
def StatusUpdate(msg):
    global movebaseStatus 
    movebaseStatus = msg
		
# Interrupt event for commanding the brain
def ExecuteCommand(msg):
	global slowDown

	currentCommand = int(msg.data)

	if currentCommand == Command.SetRandomGoal:
		SetRandomGoal()
	if currentCommand == Command.CancelGoal:
		CancelGoals()
	if currentCommand == Command.SlowDown:
		slowDown = True
	if currentCommand == Command.SpeedUp:
		slowDown = False
		
	print("Current command is: %d" % currentCommand)

# Interrupt event for changing the zone
def ExecuteChangeZone(msg):
	global currentZone

	currentZone = int(msg.data)
	print("Current zone is: %d" % currentZone)

# Interrupt event for emergency topic
def EmergencyInputCallback(msg):
	global lastEmergencyMsg

	print("Got emergency bool update")
	lastEmergencyMsg = msg
	

# Returns a location by index
def GetLocation(index):
    return tagLocations.values()[index]

# Picks a random location out of the apriltag dictionary
def GetRandomLocation():
    index = random.randint(0,len(tagLocations)-1)
    return tagLocations.values()[index]

# Returns a MoveBasGoal by location
def GetGoal(location):
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

# Returns the desirable speed of a Twist message
def GetSpeed(msg):
	if slowDown:
		msg.linear.x /= 2
	return msg

# Returns a MoveBaseGoal with a random location
def GetRandomGoal():
    return GetGoal(GetRandomLocation())

# Publish a random goal
def SetRandomGoal():
    SetGoal(GetRandomGoal())


# Publish a goal on the move_base/goal topic
def SetGoal(goal):
    global lastGoalMsg

    CancelGoals()
    lastGoalMsg = goal
    goalTopic.publish(goal)
    print("Goal set")

# Publish a cancelevent to the move_base/cancel
def CancelGoals():
    cancelTopic.publish(GoalID())
    print("Goals canceled")

# Starts autonomous movement
def SetAutonomousMovementGoal():
    SetGoal(GetGoal(GetLocation(random.randint(0, len(tagLocations)-1))))	

# Start zone movement
def SetZoneMovementGoal():
    rangeLength = (len(tagLocations)-1)/3
    rangeMin = (currentZone * rangeLength) - rangeLength
    rangeMax = (currentZone * rangeLength)
    index = random.randint(rangeMin, rangeMax)
    SetGoal(GetGoal(GetLocation(index)))

# Callback method for joystick topic
def JoystickInputCallback(msg):
    global lastJoystickMsgUpdate
    global lastJoystickMsg

    print("Got joystick input")
    lastJoystickMsg = msg
    lastJoystickMsgUpdate = time.time()

# Callback method for move_base topic
def MoveBaseInputCallback(msg):
    global lastMoveBaseMsg

    print("Got move_base cmd vel update")
    lastMoveBaseMsg = msg

# Callback method for social interaction is active topic
def SocialInteractionIsActiveCallback(msg):
	global socialInteractionActive
	if msg.data == 1 :
		socialInteractionActive = True
	else:
		socialInteractionActive = False

# Callback method for human_dect topic
def HumanDetectionInputCallback(msg):
	global lastHumanDetectionUpdate

	if "," in msg.data:
		data = tuple(map(int, msg.data.split(",")))
		if data[2] < 250:
			slowDown = True
			lastHumanDetectionUpdate = time.time()
			print("Human detected, slow down")

# Callback method for checking the current pose of willy
def CurrentPoseCallback(msg):
	global lastGoalMsg

	if IsInRange(msg.pose.pose.position.x, lastGoal.pose.pose.position.x, 1) and IsInRange(msg.pose.pose.position.y, lastGoal.pose.pose.position.y, 1):
		print("Goal reached!")

# Callback method for checking the initial pose set on move_base
def InitialPoseCallback(msg):
	global lastGoalMsg

	print("Current pose")
	print(lastGoalMsg.pose.pose.orientation)
	print("Initial pose")
	print(msg.pose.pose.orientation)
	


###############################################################

# Init ROS components
rospy.init_node("brain")

# Manual command topics
commandTopic = rospy.Subscriber("brain_command", Int32, ExecuteCommand)
zoneTopic = rospy.Subscriber("brain_change_zone", Int32, ExecuteChangeZone)

# Publisher topics
motorTopic = rospy.Publisher("cmd_vel", Twist, queue_size=25)
goalTopic = rospy.Publisher("move_base/goal", MoveBaseActionGoal,queue_size=25)
cancelTopic = rospy.Publisher("move_base/cancel", GoalID, queue_size=25)

# Subscribers
statusTopic = rospy.Subscriber("move_base/status", GoalStatusArray, StatusUpdate)
currentPoseTopic = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, CurrentPoseCallback)
initialPoseTopic = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, InitialPoseCallback)
moveBaseTopic = rospy.Subscriber("cmd_vel_move_base", Twist, MoveBaseInputCallback)
joystickTopic = rospy.Subscriber("cmd_vel_joy", Twist, JoystickInputCallback)
emergencyTopic = rospy.Subscriber("emergency", Bool, EmergencyInputCallback)
socialInteractionTopic = rospy.Subscriber("interaction/is_active", Int32, SocialInteractionIsActiveCallback)
humanDetectionTopic = rospy.Subscriber("human_dect", String, HumanDetectionInputCallback)


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
	UpdateState()
    
	if currentState == State.HumanControl:
		HandlerStateHumanControl()
	if currentState == State.Autonomous:
		HandleStateAutonomous()
	if currentState == State.Emergency:
		HandlerStateEmergency()
	if currentState == State.SocialInteraction:
		HandleSocialInteraction()
	if currentState == State.MoveToTag:
	    HandleMoveToTag()

	print(slowDown)
    
	sleep(0.1)

