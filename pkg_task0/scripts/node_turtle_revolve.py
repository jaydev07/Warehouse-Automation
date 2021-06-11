#!/usr/bin/env python

# To move the turtle inside the turtlesim window in a circle and stop at its initial location.

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

#Created a Global variable which is used to  get theta value from subcriber
myTheta=None

def rotate_turtle():

    #creating publisher for velocity 
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    
    #Created node-"node_turtle_revolve":
    rospy.init_node('node_turtle_revolve', anonymous=True)
    
    #Initializing "Speed & Radius" parameters for performing circular motion
    speed = 1
    radius = 1

    #Initializing "Linear & Angular" variables for motion
    vel_msg.linear.x = abs(speed)
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = speed/radius

    #Inital Time "t0"
    t0 = rospy.Time.now().to_sec()

    #Created current distance variable for detecting current distance of turtle:
    current_distance=0
        
    #Loop for performing a part of right circle & avoiding discrepancy of theta = 0   
    while(1):

        #publishing the current velocity msg
        velocity_publisher.publish(vel_msg)

        #subscribing the msg from publisher
	rospy.Subscriber("/turtle1/pose", Pose, pose_callback)    
	    
        #assigining the current time to variable "t1"
        t1=rospy.Time.now().to_sec()

        #Calculating the current distance 
        current_distance = speed*(t1-t0) 

        #displaying the current distance       	
	rospy.loginfo("Moving in a circle")
        print(current_distance)
        
        #Terminating the loop at given condition
        if(myTheta>0):
           break


    #Loop for performing the right half of the circle
    while(1):
        velocity_publisher.publish(vel_msg)
	rospy.Subscriber("/turtle1/pose", Pose, pose_callback)    
	t1=rospy.Time.now().to_sec()
        current_distance = speed*(t1-t0)        	
	rospy.loginfo("Moving in a circle")
        print(current_distance)
        
        if(myTheta<0):
	   break

    #Loop for performing the left half of the circle
    while(1):
        velocity_publisher.publish(vel_msg)
	rospy.Subscriber("/turtle1/pose", Pose, pose_callback)    
	t1=rospy.Time.now().to_sec()
        current_distance = speed*(t1-t0)        	
	rospy.loginfo("Moving in a circle")
        print(current_distance)

        if(myTheta>0):

          #Acknowledgement  
          rospy.loginfo("goal reached")
          break
    
      
    velocity_publisher.publish(vel_msg)

    #Rotating turtle on its axis
    vel_msg.angular.z = 1.5
    vel_msg.linear.x=0
    velocity_publisher.publish(vel_msg)

#Callback function for subscriber
def pose_callback(msg):

    #Storing the theta value inside myTheta
    global myTheta
    myTheta=msg.theta

#Starting of main function
if __name__ == '__main__':
    try:
        rotate_turtle()
    except rospy.ROSInterruptException: pass
