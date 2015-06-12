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

def main():
#	rospy.init_node('fontys_demo', anonymous=True)
	rospy.init_node('fontys_demo', anonymous=False)
	#Reading Sensors
	#read joint angles
	print baxter_interface.Limb('right').joint_angles()
	#read button state
	print baxter_interface.digital_io.DigitalIO('right_lower_button').state
	#read IR rangefinder
	print baxter_interface.analog_io.AnalogIO('right_hand_range').state()

while 0 == 0:
	print("something")



if __name__ == '__main__':
    main()
