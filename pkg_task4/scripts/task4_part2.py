#! /usr/bin/env python

import rospy
import sys
import copy
import time
import yaml

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import rospkg

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.msg import LogicalCameraImage

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse

from pkg_task4.msg import myMessage
from pkg_task4.msg import myModel

box_length = 0.15               # Length of the Package
vacuum_gripper_width = 0.115    # Vacuum Gripper Width
delta = vacuum_gripper_width + (box_length/2)  # 0.19
        # Teams may use this info in Tasks

ur5_2_home_pose = geometry_msgs.msg.Pose()
ur5_2_home_pose.position.x = -0.8
ur5_2_home_pose.position.y = 0
ur5_2_home_pose.position.z = 1 + vacuum_gripper_width + (box_length/2)
        # This to keep EE parallel to Ground Plane
ur5_2_home_pose.orientation.x = -0.5
ur5_2_home_pose.orientation.y = -0.5
ur5_2_home_pose.orientation.z = 0.5
ur5_2_home_pose.orientation.w = 0.5

message_reveived = False

global lst_joint_angles_1
global lst_joint_angles_2
global lst_joint_angles_3
global lst_joint_angles_4

# Array which will store the Message coming in Subscriber
subscriber_msg = []

# Array which will story the models captured by Logical Camera 2
pkg_detected_byLc = []

####################################################################################### Ur5 CartesianPath ###################################################################################################

class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('task4_part2', anonymous=True)

        self._robot_ns = '/'  + 'ur5_2'
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''


        # Attribute to store computed trajectory by the planner	
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if (flag_plan == True):
            pass
            # rospy.loginfo(
            #     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()


    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name
        
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # # self.clear_octomap()
        
        return True

        
    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


#################################################################################### Subscriber Function ####################################################################################################

def function_callback(msg):
   global subscriber_msg

   # Storing the message which contains the pair of package name and colour into subscriber_msg
   subscriber_msg = msg
   print("SUBS MSG: ",subscriber_msg)  
   print("\n")


####################################################################################### Logical Camera 2 ####################################################################################################

def logical_camera2_callback_function(msg):
  global pkg_detected_byLc

  # Storing the models detected by Logical Camera 2 in pkg_detected_byLc array
  pkg_detected_byLc= msg.models
  


######################################################################################## Main Function ######################################################################################################

def main():

  # ur5_2 Controller
  ur5 = Ur5Moveit()
  
  # Subscribing "my_topic"
  rospy.Subscriber("my_topic" , myMessage , function_callback)

  # Subscribing "/eyrc/vb/logical_camera_2"
  rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, logical_camera2_callback_function)  

  # Main thread should be in sleep mode untile the message is not received by Subscriber
  time.sleep(7)

  # Moving UR5_2 to home pose
  ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_home_0.yaml', 5)

  global pkg_detected_byLc
  global subscriber_msg

  while not rospy.is_shutdown():

        # When the package is detected by Logical Camera 2
	if(len(pkg_detected_byLc)>1):

		# Taking the package 
		for pkg in pkg_detected_byLc:
			print("PACKAGE DETECTED BY LC1: ",pkg.type)
	
			# Traversing through the pair of colour & package name from subscriber_msg.my_model
			for model in subscriber_msg.my_model:
		
				# Converting a string into an array by replace & split methods( Eg:- message=['red','packagen00'] )		
                                bra_removed = model.replace(']','').replace('[','')
				message = bra_removed.replace('"','').split(",")
				print("MODEL MSG IN SUB MSG: ",message)

				# Checking that the package detected by LC2 is present in subscriber message
				if(pkg.type == message[1]):
					print("MATCHED PACKAGE")

					# Checking that the package is in the center position & ready to pick 
					if(pkg.pose.position.y / 0.046 < 0 ):
						print("package in center position")

						# Activating the vaccume gripper to pick up the package
						v = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
				   		v(True)		
				       		print("NOW PICKING BLOCK")

						# Checking that the package is of which colour so that we can place the package into it's respected coloured box
						if(message[0] == "red"):
							print("NOW WE CAN MOVE TO RED BOX")
							# Move to RED BOX
							ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_red.yaml', 5)

							# Deactivating vacuum gripper
			 				v(False)
								
							# Moving back to home pose	
							ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_home_r.yaml', 5)

						elif(message[0] == "green"):
							print("NOW WE CAN MOVE TO GREEN BOX")
							# Move to GREEN BOX
							ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_green.yaml', 5)
			 				v(False)
							ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_home_g.yaml', 5)

						elif(message[0] == "yellow"):
							print("NOW WE CAN MOVE TO YELLOW BOX")
							# Move to YELLOW BOX
							ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_yellow.yaml', 5)
		 					v(False)
							ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_home_y.yaml', 5)


if __name__ == '__main__':
    main()
