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

def moveto(pose):
	print("this is the move to function")
	print(pose)


def main():
	rospy.init_node('fontys_demo', anonymous=True)
	#Reading Sensors
	#read joint angles
	print baxter_interface.Limb('left').joint_angles()
	#read button state
	print baxter_interface.digital_io.DigitalIO('right_lower_button').state
	#read IR rangefinder
	print baxter_interface.analog_io.AnalogIO('right_hand_range').state()

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



	startpose = [0.6, 0.8, 0.1, math.pi, 0, math.pi]
	moveto(startpose)

	rospy.spin()



if __name__ == '__main__':
    main()
