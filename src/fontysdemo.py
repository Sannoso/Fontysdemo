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
	#convert the Eulers RPY into quaternion xyzw
	quaternion_pose = conversions.list_to_pose_stamped(input_pose, "base")
	#starting the IK request
	node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
	ik_service = rospy.ServiceProxy(node, SolvePositionIK)
	ik_requestmessage = SolvePositionIKRequest()
	ik_requestmessage.pose_stamp.append(quaternion_pose)

	# QUICK & DIRTY: MOLT THIS INTO AN TRY and EXCEPT handler!!
	rospy.wait_for_service(node, 5.0)
	ik_response = ik_service(ik_requestmessage)
	if (ik_response.isValid[0]):
		print("Jay! Valid joint solution found")
		#convert response to JP control dict
		limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
		return limb_joints
	else:
		print("FAILED: No valid joint configuration for this pose found")
	

def MoveToPose(limb, targetpose): #NOTE ERRORHANDLING MUST STILL BE DONE!!
	#first calculate IK	
	angles = solve_ik('left', targetpose)
	#IK found so now move
	baxter_interface.Limb(limb).move_to_joint_positions(angles)
	
def GripperClose(limb):
	baxter_interface.Gripper(limb).close()
	rospy.sleep(1)

def GripperOpen(limb):
	baxter_interface.Gripper(limb).open()
	rospy.sleep(1)

def CheckForObject(limb):
	camera_sub = rospy.Subscriber("/cameras/right_hand_camera/image",Image,callback)
	
def InitialiseRobot():
	#Enable the actuators
	baxter_interface.RobotEnable().enable()
	
	#check if gripper is calibrated, if not, reboot it and calibrate
	print "gripper calibrated: ",baxter_interface.Gripper('right').calibrated()
	if baxter_interface.Gripper('right').calibrated() == False:
	    print "calibrating now"
	    #baxter_interface.Gripper('right').reboot()
	    baxter_interface.Gripper('right').calibrate()

def callback(data):

    #Republish the camera stream to the screen
    rospy.Publisher('/robot/xdisplay',Image, queue_size=1).publish(data)

    #Convert incoming image from a ROS image message to a CV image that open CV can process.
    cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    #Display the converted cv image, this is the raw camera feed data.
    cv2.imshow("Raw Camera Feed", cv_image)

    #Create an empty image variable, the same dimensions as our camera feed.
    gray = numpy.zeros((cv_image.shape), numpy.uint8)
    #Into this previously created empty image variable, place the grayscale conversion of our camera feed.
    gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)

    #Create another empty image variable.
    canny = numpy.zeros((cv_image.shape), numpy.uint8)
    #Fill the new image variable with a canny edge detection map of the greyscale image created earlier.
    canny = cv2.Canny(gray, 50, 150, 3)
    #Display the canny mapping.
    cv2.imshow("Canny Edge Detection", canny)

    #3ms wait
    cv2.waitKey(3)

def main():
	rospy.init_node('fontys_demo', anonymous=True)
	InitialiseRobot()

	#defining all poses
	startposeleftarm = [0.6, 0.85, 0.2, math.pi, 0, math.pi]
	pickpose_down_leftarm = [0.6, 0.85, 0.01, math.pi, 0, math.pi]
	pickpose_up_leftarm = [0.6, 0.85, 0.2, math.pi, 0, math.pi]
	placepose_down_leftarm = [0.45, 0.60, 0.05, math.pi, 0, math.pi]
	placepose_up_leftarm = [0.45, 0.60, 0.10, math.pi, 0, math.pi]

	#create subscriber to the right hand camera, each frame recieved calls the callback function
	camera_sub = rospy.Subscriber("/cameras/right_hand_camera/image",Image,callback)

	MoveToPose("left", startposeleftarm)
	MoveToPose("left", pickpose_down_leftarm)
	print("debug, gripperclose now:")
	GripperOpen("left")
	rospy.sleep(3) 
	#somehow I need to call this open function for the close function to have effect
	GripperClose("left")
	MoveToPose("left", pickpose_up_leftarm)
	MoveToPose("left", placepose_up_leftarm)
	MoveToPose("left", placepose_down_leftarm)
	GripperOpen("left")
	MoveToPose("left", placepose_up_leftarm)
	print("spinning now")
	
	rospy.spin()

if __name__ == '__main__':
    main()
