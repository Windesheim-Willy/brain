#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

# Starts a new node
rospy.init_node('move_willy', anonymous=True)
velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
vel_msg = Twist()


def question():
    #Receiveing the user's input
    print("Wat wil je Willy laten doen? Antwoord binair!")
    moveForward = input("Wil je willy vooruit laten rijden?")
    if(moveForward):
        isForward = True
        move(isForward)
    else:
        moveBackward = input("Wil je willy achteruit laten rijden?")
        if(moveBackward):
            isForward = False
            move(isForward)
        else:
            moveLeft = input("Wil je Willy een bocht naar links laten rijden?")
            if(moveLeft):
                isLeft = True
                turn(isLeft)
            else:
                moveRight = input("Wil je Willy een bocht naar rechts laten rijden?")
                if(moveRight):
                    isLeft = False
                    turn(isLeft)                    
                else:
                    print("Geen keuze gemaakt, probeer opnieuw!")
                    question()


def move(isForward):
    r = rospy.Rate(10) # 10hz 
    
    #Receiveing the user's input
    print("Je wilt Willy laten rijden, here we go!")
    speed = input("Snelheid:")
    distance = input("tijd:")
    #isForward = input("Vooruit?: ")#True or False
    
    #Checking if the movement is forward or backwards
    if(isForward):
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
    
        
    #Since we are moving just in x-axis
    #vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    while not rospy.is_shutdown():

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
            r.sleep()
        #After the loop, stops the robot
        while(x<5):
            x = x+1
            vel_msg.linear.x = 0.0
            velocity_publisher.publish(vel_msg)
            r.sleep()
        #Force the robot to stop
        
        question()

def turn(isLeft):
    r = rospy.Rate(10) # 10hz 

    #Receiveing the user's input
    print("Je wilt Willy een bocht laten maken, here we go!")
    speed = input("Snelheid:")
    distance = input("Afstand:")
    #isForward = input("Vooruit?: ")#True or False
    
    #Checking if the movement is forward or backwards
    if(isLeft):
        vel_msg.angular.z = abs(speed)
    else:
        vel_msg.angular.z = -abs(speed)
    
        
    #Since we are moving just in x-axis
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    # vel_msg.angular.z = 0
    
    while not rospy.is_shutdown():

        
        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
            r.sleep()
        #After the loop, stops the robot
        while(x<5):
            x = x+1
            vel_msg.angular.z = 0.0
            velocity_publisher.publish(vel_msg)
            r.sleep()
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)
        question()

def myhook():
  print "shutdown time!"
rospy.on_shutdown(myhook)

if __name__ == '__main__':
    try:
        #Testing our function
        question()
    except rospy.ROSInterruptException: pass
