#! /usr/bin/env python

import rospy
import sys
import copy
import time

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.msg import LogicalCameraImage

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse

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

count=-1

my_pkgs = []

class CartesianPath:

    # Constructor
    def __init__(self):

        rospy.init_node('node_t3', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')


    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))


        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    def go_to_pose(self, arg_pose):
    

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan
 
    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan


    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')

    def camera_callback_function(self,msg):
	global my_pkgs
	global count
	my_pkgs= msg.models
	for pkg in my_pkgs:
		print(pkg.type)
	c = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',conveyorBeltPowerMsg)
        if(count>0):	
		c(0)
        elif(count==0):
		c(19)
	elif(count==-1):
		c(100)
	elif(count==-2):
		c(21)

	
def main():
    ur5 = CartesianPath()
    rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, ur5.camera_callback_function)
    ur5.go_to_pose(ur5_2_home_pose)
    global count 
    global my_pkgs
    lst_joint_angles_1 = [math.radians(-30),
                          math.radians(-30),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]

    lst_joint_angles_2 = [math.radians(-100),
                          math.radians(0),
                          math.radians(-35),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]
	

    while not rospy.is_shutdown():
	if(len(my_pkgs)>0):
		for pkg in my_pkgs:
			if(pkg.type == "packagen1"):
				print("RED FIRST IF")
				if(pkg.pose.position.y / 0.046 < 0 ):
					print("SECOND IF")
					count=1
					v = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
		    			v(True)
					print("INSIDE RED")		
		               		print("NOW PICKING RED BLOCK")
					count=0
					ur5.ee_cartesian_translation(1.5, 1, 0.5)
					#v = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
	 				v(False)
					ur5.go_to_pose(ur5_2_home_pose)
                	
			elif(pkg.type == "packagen2"):
				print("GREEN FIRST IF")
				if(pkg.pose.position.y / 0.046 < 0 ):
					print("GREEN SECOND IF")
					count=1
					print("INSIDE GREEN")		
					#ur5.go_to_pose(ur5_2_home_pose)
					ur5.ee_cartesian_translation(0.2,0,0)
					v = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
		    			v(True)
		               		print("NOW PICKING GREEN BLOCK")
					count=-2
                                        #ur5.go_to_predefined_pose("straightUp")
                                        #ur5.go_to_predefined_pose("green")
					ur5.set_joint_angles(lst_joint_angles_1)  
					#v = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
	 				v(False)
					ur5.go_to_pose(ur5_2_home_pose)
					
	
			elif(pkg.type == "packagen3"):
				print("BLUE FIRST IF")
				print("Y'S VALUE : ",pkg.pose.position.y)
				if(pkg.pose.position.y / 0.037 < 0 ):
					print("BLUE SECOND IF")
					count=1
					print("INSIDE BLUE")		
					#ur5.go_to_pose(ur5_2_home_pose)
		               		ur5.ee_cartesian_translation(-0.1,0, 0)
					v = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
		    			v(True)
					print("NOW PICKING BLUE BLOCK")
					#count=1
					ur5.set_joint_angles(lst_joint_angles_2)
					#ur5.ee_cartesian_translation(1.5,-1, 0.4)
					#v = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
	 				v(False)
					ur5.go_to_pose(ur5_2_home_pose)
	
	
    del ur5


if __name__ == '__main__':
    main()
