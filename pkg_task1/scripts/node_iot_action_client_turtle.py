#!/usr/bin/env python

# ROS Node - Simple Action Client - Turtle

import rospy
import actionlib
import time
import requests

#importing msgTurtle from pkg_task1 package
from pkg_task1.msg import msgTurtleAction       # Message Class that is used by ROS Actions internally
from pkg_task1.msg import msgTurtleGoal         # Message Class that is used for Goal messages

#importing msgMqttSub from pkg_ros_iot_bridge package
from pkg_ros_iot_bridge.msg import msgMqttSub

#importing msgRosIotAction from pkg_ros_iot_bridge package
from pkg_ros_iot_bridge.msg import msgRosIotAction      # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal        # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult      # Message Class that is used for Result Messages

#Declaring variable to get acknoledgment when the result is arrived from Action Client
result_from_ac=None

#Using Counter for completing Hexagon
counter=1


#Making a class of Action Client
class IotRosBridgeActionClient:

    # Constructor
    def __init__(self):

        # Initialize Action Client
        self._ac = actionlib.ActionClient('/action_ros_iot',
                                          msgRosIotAction)
        
        # Dictionary to Store all the goal handels
        self._goal_handles = {}

        # Store the MQTT Topic on which to Publish in a variable
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']

        # Wait for Action Server that will use the action - '/action_iot_ros' to start
        self._ac.wait_for_server()
        rospy.loginfo("Action server up, we can send goals.")

    
    # This function will be called when there is a change of state in the Action Client State Machine
    def on_transition(self, goal_handle):
        
        # from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.
        
	global result_from_ac
        result = msgRosIotResult()

        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
                index = i
                break

        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()) )
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()) )
        
        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done
        
        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")
        
        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())
            
            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)
            
	    #Storing result.flag_success in our global variable result_from_ac
	    result_from_ac=result.flag_success
            if (result.flag_success == True):
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
                
            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))


    # This function is used to send Goals to Action Server
    def send_goal(self, arg_protocol, arg_mode, arg_topic, arg_x,arg_y,arg_theta):
        # Create a Goal Message object
        goal = msgRosIotGoal()

        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.turtle_x=arg_x
	goal.turtle_y=arg_y
	goal.turtle_theta=arg_theta

        rospy.loginfo("Send goal.")
        
        # self.on_transition - It is a function pointer to a function which will be called when 
        #                       there is a change of state in the Action Client State Machine
        goal_handle = self._ac.send_goal(goal,
                                         self.on_transition,
                                         None)

        return goal_handle



#Creating a class of Simple Action Client
class SimpleActionClientTurtle:

    # Constructor
    def __init__(self):
        self._ac = actionlib.SimpleActionClient('/action_turtle',
                                                msgTurtleAction)
        self._ac.wait_for_server()
        rospy.loginfo("Action server is up, we can send new goals!")
	

    # Function to send Goals to Action Servers
    def send_goal(self, arg_dis, arg_angle):
        
        # Create Goal message for Simple Action Server
        goal = msgTurtleGoal(distance=arg_dis, angle=arg_angle)
        
        '''
            * done_cb is set to the function pointer of the function which should be called once 
                the Goal is processed by the Simple Action Server.

            * feedback_cb is set to the function pointer of the function which should be called while
                the goal is being processed by the Simple Action Server.
        ''' 
        self._ac.send_goal(goal, done_cb=self.done_callback,
                           feedback_cb=self.feedback_callback)
        
        rospy.loginfo("Goal has been sent.")

    # Function print result on Goal completion
    def done_callback(self, status, result):
        rospy.loginfo("Status is : " + str(status))
        rospy.loginfo("Result is : " + str(result))


        # After getting Result from Simple Action Server we are creating object of Action Client
	action_client = IotRosBridgeActionClient()

        # Sending goal from Action Client to Action Server  
    	goal_handle1 = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, str(result.final_x),str(result.final_y),str(result.final_theta))
    	action_client._goal_handles['1'] = goal_handle1
    	rospy.loginfo("Goal Sent")

       
        # This thread sleeps for 1 second
        time.sleep(4)

        # Using Counter to make a Hexagon
        global counter
	
        rospy.loginfo(result_from_ac )

        # Checks the result arrived from Action Client 
	if(result_from_ac==True and counter<6):
	    rospy.loginfo("Want to send new goal")

            # Sending the new goal to Simple Action Server to move turtle in turtlesim
	    self.send_goal(2, 60)
	    
            # Incrementing the counter 
	    counter+=1
    	    rospy.sleep(5)
        


    # Function to print feedback while Goal is being processed
    def feedback_callback(self, feedback):
        rospy.loginfo(feedback)

    # Callback Function which executes when a message is arrived in /ros_iot_bridge/mqtt/sub topic
    def func_ros_sub_callback(self,msg):
        print(msg)
	self.received=msg.message

        # To initially move turtle when "start" message is arrived in /ros_iot_bridge/mqtt/sub topic
        if(self.received=="start"):
    		self.send_goal(2, 0)
    		rospy.sleep(5)


# Main Function
def main():
    # 1. Initialize ROS Node
    rospy.init_node('node_iot_action_client_turtle')

    # Creating object for Simple Action Client
    obj_client = SimpleActionClientTurtle()
    
    # Subscribing on /ros_iot_bridge/mqtt/sub topic    
    handle_sub_ros_iot_bridge = rospy.Subscriber("/ros_iot_bridge/mqtt/sub", msgMqttSub, obj_client.func_ros_sub_callback)
   
    # 4. Loop forever
    rospy.spin()


if __name__ == '__main__':
    main()
