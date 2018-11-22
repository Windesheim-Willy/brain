#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import String

a = '0'
b = 0
c = 0
d = '24'

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global a
    y = data.data
    # a = tag_nummer
    # b = graden
    # c = Windstreek
    a,b,c = y.split(",")
    #print(a)
    
    

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("openmv_apriltag", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

# Starts a new node
rospy.init_node('move_willy', anonymous=True)
velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=2)
vel_msg = Twist()
#rospy.init_node('listener', anonymous=True)
#rospy.Subscriber("openmv_apriltag", String, callback)




def question():
    #Receiveing the user's input
    print("Tags")
    d = input("Bij welke Tag moet Willy stoppen?")
    d = str(d)
    isForward = True
    move(isForward)


def move(isForward):
    r = rospy.Rate(10) # 10hz 
    x = 0 
    
    #Receiveing the user's input
    print("Je wilt Willy laten rijden, here we go!")
    speed = 0.4
    distance = 1
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
        while(a != d):
            listener()
            #print(a)
            #print(d)
            print('Rijden')
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
            r.sleep()
        #After the loop, stops the robot
        while(x<5):
            print('Stoppen')
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
