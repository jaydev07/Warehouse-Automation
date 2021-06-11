#!/usr/bin/env python

import rospy
import cv2
import sys

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg

import yaml
import os
import math
import time
import copy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from hrwros_gazebo.msg import LogicalCameraImage
from cv_bridge import CvBridge, CvBridgeError

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse

from pkg_vb_sim.srv import conveyorBeltPowerMsg

from std_srvs.srv import Empty

from pyzbar.pyzbar import decode

from pkg_task4.msg import myMessage
#from pkg_t2_examples.msg import myModel 

################################## Stored Colors ###########################

# Array which contains the pair of colours & their respective packages(Eg:- ['red','packagen00'])
all_components = []

# Array which contains all 9 packages which shoul be picked by ur5_1(Eg:- ["packagen00','packagen01',...]
pkgs_array=[]

# Variable which will become True when all the pair are stored in "all_components"
msg_received = False

################################## For Motion ###########################

# Stores the package for a perticular color
package_found=None

# Variable used to capture an image from camera#1 only for one time
msg_ready = True

################################################################################  Ur5_moveit Class ##################################################################################################

class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('task4_part1', anonymous=True)

        self._robot_ns = '/'  + 'ur5_1'
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


################################################################################  Camera Class ##################################################################################################

class Camera1:

  def __init__(self):
    self.bridge = CvBridge()
    
    # Subscribing the topic "/eyrc/vb/camera_1/image_raw" for getting an image stream
    self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)

  
  def get_qr_data(self, arg_image):

    # Decoding the stream into an image
    qr_result = decode(arg_image)

    global all_components
    global pkgs_array

    global msg_received

    global package_found
    global msg_ready

    # Taking the  decoded data
    if ( len( qr_result ) > 0 and msg_ready == True):

      # Changing msg_ready variable so that camera#1 can capture the image only once
      msg_ready = False

      # Traversing through every package detected by camera#1
      for i in range(0,len(qr_result)):

        # Taking the dimensions and location of a perticular package with the help of openCV
	r = qr_result[i].rect

	left = r.left
	top = r.top
	width = r.width
	height = r.height

	# Assigning the package name to a box detected
	if(top>300 and top<400):
		if(left>100 and left<300):
			package_found = "packagen00"
		if(left>300 and left<420):
			package_found = "packagen01"
		if(left>420 and left<600):
			package_found = "packagen02"

	elif(top>400 and top<520):
		if(left>100 and left<300):
			package_found = "packagen10"
		if(left>300 and left<420):
			package_found = "packagen11"
		if(left>420 and left<600):
			package_found = "packagen12"
	
	elif(top>520 and top<670):
		if(left>100 and left<300):
			package_found = "packagen20"
		if(left>300 and left<420):
			package_found = "packagen21"
		if(left>420 and left<600):
			package_found = "packagen22"
		
	elif(top>670 and top<820):
		if(left>100 and left<300):
			package_found = "packagen30"
		if(left>300 and left<420):
			package_found = "packagen31"
		if(left>420 and left<600):
			package_found = "packagen32"

	# Storing the package name into "pkgs_array"
        pkgs_array.append(package_found)

	# Storing the pair of colour and package name into all_components array
	single_component="["+qr_result[i].data+","+package_found+"]"
	all_components.append(single_component)

      # Changing the variable msg_received because all the desired data is stored in the arrays 
      msg_received = True
      
  
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)

    (rows,cols,channels) = cv_image.shape
    
    # Saving the image
    image = cv_image

    # Grayscaling the image
    image_grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Converting the image into threshold value so that it can decode all the packages
    image_thr = cv2.adaptiveThreshold(image_grey,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)

    # Using Contours 
    im, contours , hierarchy = cv2.findContours(image_thr, cv2.RETR_TREE , cv2.CHAIN_APPROX_SIMPLE)

    with_contours = cv2.drawContours(image, contours,-1,(0,255,0),3)

    # Resize a 720x1280 image to 360x640 to fit it on the screen
    resized_image = cv2.resize(with_contours, (720/2, 1280/2)) 
    cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)    
    
    # Calling get_qr_data function
    self.get_qr_data(image_thr)
    
    cv2.waitKey(3)



#####################################################  Main Function #################################################################################################


def main():

  # Initializing the Node
  #rospy.init_node('task4_part1', anonymous=True)

  # ur5_1 arm controller
  ur5 = Ur5Moveit()
  
  global msg_received
  global all_components

  global pkgs_array

  # Publisher which will publish all_components array
  pub = rospy.Publisher('my_topic', myMessage , queue_size=10)

  # Message which will be published
  my_msg = myMessage()

  # Initializing Camera1 Class
  time.sleep(9)
  ic = Camera1()

  # Moving the conveyor belt with the speed of 100 
  c = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',conveyorBeltPowerMsg)
  c(100)

  # Main thread should be in sleep mode untile all the data is stored from Camera1 class
  time.sleep(1.5)

  # Checking that all the data is stored in arrays or not
  if(msg_received == True):

	# Assiging all_components array to my_msg which contains the pair of package name and the color of that package
	my_msg = all_components

        # Publishing the message
	pub.publish(my_msg)
	print("\n")
	print("Published: ",my_msg)

	# Traversing through all the packages and picking them one-by-one using ur5_1
	for package_found in pkgs_array:
		print("PKG: ",package_found)

		if(package_found == "packagen31" or package_found == "packagen32" or package_found == "packagen22"):
			continue
		else:
			## Motion ##

			############## 1nd Row #################

			if(package_found == "packagen00"):

				# Moving ur5_1 towards package
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_00_2.yaml', 5)

				# Activating Vacuume Gripper
				v = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
		    		v(True)

				# Taking out the package
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_00_3.yaml', 5)

				# Placing the package on the conveyor belt
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_home_00.yaml', 5)	

			elif(package_found == "packagen01"):
		    		ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_01_2.yaml', 5)
				v = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
		    		v(True)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_01_3.yaml', 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_home_01.yaml', 5)	
		    	
			elif(package_found == "packagen02"):
		    		ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_02_2.yaml', 5)
				v = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
		    		v(True)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_02_3.yaml', 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_home_02.yaml', 5)	


			############## 2nd Row #################

			elif(package_found == "packagen10"):
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_10_2.yaml', 5)
				v = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
		    		v(True)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_10_3.yaml', 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_home_10.yaml', 5)	

			elif(package_found == "packagen11"):
		    		ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_11_2.yaml', 5)
				v = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
		    		v(True)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_11_3.yaml', 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_home_11.yaml', 5)	

			elif(package_found == "packagen12"):
		    		ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_12_2.yaml', 5)
				v = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
		    		v(True)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_12_3.yaml', 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_home_12.yaml', 5)	
			

			############## 3rd Row #################

			elif(package_found == "packagen20"):
		    		ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_20_2.yaml', 5)
				v = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
		    		v(True)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_20_3.yaml', 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_home_20.yaml', 5)	

			elif(package_found == "packagen21"):
		    		ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_21_2.yaml', 5)
				v = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
		    		v(True)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_21_3.yaml', 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_home_21.yaml', 5)	
						
			############## 3rd Row #################

			elif(package_found == "packagen30"):
		    		ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_30_2.yaml', 5)
				v = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
		    		v(True)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_30_3.yaml', 5)
				ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'lst_joint_angles_home_30.yaml', 5)	

			# Deactivating the vacuum gripper so that it can place the package on conveyor belt
			v = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)		
			v(False)




  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
