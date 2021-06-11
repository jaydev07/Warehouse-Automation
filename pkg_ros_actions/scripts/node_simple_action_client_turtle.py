#!/usr/bin/env python

# ROS Node - Simple Action Client - Turtle

import rospy
import actionlib
import time

from pkg_ros_actions.msg import myActionMsgAction       # Message Class that is used by ROS Actions internally
from pkg_ros_actions.msg import myActionMsgGoal         # Message Class that is used for Goal messages


class SimpleActionClientTurtle:

    # Constructor
    def __init__(self):
        self._ac = actionlib.SimpleActionClient('/action_turtle',
                                                myActionMsgAction)
        self._ac.wait_for_server()
        rospy.loginfo("Action server is up, we can send new goals!")

    # Function to send Goals to Action Servers
    def send_goal(self, arg_dis, arg_angle):
        
        # Create Goal message for Simple Action Server
        goal = myActionMsgGoal(distance=arg_dis, angle=arg_angle)
        
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

    # Function to print feedback while Goal is being processed
    def feedback_callback(self, feedback):
        rospy.loginfo(feedback)


# Main Function
def main():
    # 1. Initialize ROS Node
    rospy.init_node('node_simple_action_client_turtle')

    # 2. Create a object for Simple Action Client.
    obj_client = SimpleActionClientTurtle()

    # 3. Send Goals to Draw a Square
    obj_client.send_goal(2, 0)
    rospy.sleep(5)

    obj_client.send_goal(2, 90)
    rospy.sleep(5)

    obj_client.send_goal(2, 90)
    rospy.sleep(5)

    obj_client.send_goal(2, 90)
    rospy.sleep(5)

    obj_client.send_goal(2, 90)

    # 4. Loop forever
    rospy.spin()


if __name__ == '__main__':
    main()

