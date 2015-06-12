#!/usr/bin/env python

"""
Sanders vanaf scratch programma
"""

import rospy
import math
import roslib
roslib.load_manifest('fontysdemo')    #we load the manifest of the package we are in
import baxter_interface
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy

from moveit_commander import conversions
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

def solve_ik(input_limb, input_pose):
#	print("IK requested for", input_limb)
	if len(input_pose) != 6:
		print("6 inputs required!")
		return
	if input_limb == "right" or input_limb == "left":
		limb = input_limb
	else:
		print """Invalid Limb:
    Input Limb to function: ik_solver_request must be a string:
    'right' or 'left'"""
	print("input pose in Eulers RPY")
	print(input_pose)
	print("input pose converted to quaternions")
	quaternion_pose = conversions.list_to_pose_stamped(input_pose, "base")
	print(quaternion_pose)
	#starting the IK request
	node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
	ik_service = rospy.ServiceProxy(node, SolvePositionIK)
	ik_requestmessage = SolvePositionIKRequest()
	ik_requestmessage.pose_stamp.append(quaternion_pose)
	print(ik_requestmessage)

	# QUICK & DIRTY: MOLT THIS INTO AN TRY and EXCEPT handler!!
	rospy.wait_for_service(node, 5.0)
	ik_response = ik_service(ik_requestmessage)
	if (ik_response.isValid[0]):
		print("Jay! Valid joint configuration found")
		#convert response to JP control dict
		limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
		return limb_joints
	else:
		print("FAILED: No valid joint configuration for this pose found")


	
	

def moveto(targetpose, limb):
	print("this is the move to function")
	print(targetpose)
	#first calculate IK	
	angles = solve_ik('left', targetpose)
	print("angles:")
	print(angles)
	#IK found so now move
	baxter_interface.Limb(limb).move_to_joint_positions(angles)
	



def main():
	rospy.init_node('fontys_demo', anonymous=True)

	#read joint angles
#	print baxter_interface.Limb('left').joint_angles()


	#Enable the actuators
	baxter_interface.RobotEnable().enable()

	#move to joint positions in radians
	print("and now for the starting position")
	baxter_interface.Limb('left').move_to_joint_positions({
                                                        'left_s0': 0.47, 
                                                        'left_s1': -0.33, 
                                                        'left_e0': -1.04, 
                                                        'left_e1': 1.00, 
                                                        'left_w0': 0.95, 
                                                        'left_w1': 1.32, 
                                                        'left_w2': -1.307 })

	print("and now for the starting position somewhat higher")
	baxter_interface.Limb('left').move_to_joint_positions({
                                                        'left_s0': 0.47, 
                                                        'left_s1': -0.63, 
                                                        'left_e0': -1.20, 
                                                        'left_e1': 0.96, 
                                                        'left_w0': 0.78, 
                                                        'left_w1': 1.62, 
                                                        'left_w2': -1.20 })

	print("now going to conveyor belt pickup position")
	baxter_interface.Limb('left').move_to_joint_positions({
                                                        'left_s0': 0.42, 
                                                        'left_s1': -0.02, 
                                                        'left_e0': -1.36, 
                                                        'left_e1': 1.32, 
                                                        'left_w0': 1.44, 
                                                        'left_w1': 1.35, 
                                                        'left_w2': -1.73 })




	startposeleftarm = [0.6, 0.8, 0.1, math.pi, 0, math.pi]
#	startposeietshoger = 
	moveto(startposeleftarm, "left")
	pickposeleftarm = [0.6, 0.8, 0.01, math.pi, 0, math.pi]
	moveto(pickposeleftarm, "left")

#	rospy.spin()



if __name__ == '__main__':
    main()
