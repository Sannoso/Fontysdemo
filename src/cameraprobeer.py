#!/usr/bin/env python

"""
Active Robots, Baxter Training Example
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





#callback function for camera subscriber, called by the camera subscriber for every frame.
def callback(data):

    #Republish the camera stream to the screen
    rospy.Publisher('/robot/xdisplay',Image, queue_size=1).publish(data)

    #Convert incoming image from a ROS image message to a CV image that open CV can process.
    cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    #Display the converted cv image, this is the raw camera feed data.
#    cv2.imshow("Raw Camera Feed", cv_image)

    #Create an empty image variable, the same dimensions as our camera feed.
    gray = numpy.zeros((cv_image.shape), numpy.uint8)
    #Into this previously created empty image variable, place the grayscale conversion of our camera feed.
    gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    #Display the grayscale image.
#    cv2.imshow("Grayscale Conversion", gray)

	

    #Create another empty image variable.
    cannygray = numpy.zeros((cv_image.shape), numpy.uint8)
    #Fill the new image variable with a canny edge detection map of the greyscale image created earlier.
    cannygray = cv2.Canny(gray, 50, 150, 3)
    #Display the canny mapping.
#    cv2.imshow("Canny Edge Detection gray", cannygray)

	#also create an HSV image. to use the Hue channel, might be better then grey
	#create empty image again
    hsv = numpy.zeros((cv_image.shape), numpy.uint8)
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	#display result
#    cv2.imshow("hsv", hsv)

    #Create another empty image variable.
    cannyhsv = numpy.zeros((cv_image.shape), numpy.uint8)
    #Fill the new image variable with a canny edge detection map of the greyscale image created earlier.
    cannyhsv = cv2.Canny(hsv, 50, 150, 3)
    #Display the canny mapping.
#    cv2.imshow("Canny Edge Detection hsv", cannyhsv)	
	
	#splitting hsv, to use
    hue = numpy.zeros((cv_image.shape), numpy.uint8)
    saturation = numpy.zeros((cv_image.shape), numpy.uint8)
    value = numpy.zeros((cv_image.shape), numpy.uint8)
    hue, saturation, value = cv2.split(hsv)
    cv2.imshow("hue", hue)
#    cv2.imshow("saturation", saturation)		
    #value is the same as gray, so don't show it again
    #cv2.imshow("value", value)	

    #Create another empty image variable.
    cannyhue = numpy.zeros((cv_image.shape), numpy.uint8)
    #Fill the new image variable with a canny edge detection map of the greyscale image created earlier.
    cannyhue = cv2.Canny(hue, 180, 230, 10)
    #Display the canny mapping.
    cv2.imshow("Canny Edge Detection hue", cannyhue)

    blurhue = numpy.zeros((cv_image.shape), numpy.uint8)
    blurhue = cv2.GaussianBlur(hue, (9,9), 0)
    cv2.imshow("blur", blurhue)
    cannyblurhue = numpy.zeros((cv_image.shape), numpy.uint8)
    cannyblurhue = cv2.Canny(blurhue, 30,150,3)
    cv2.imshow("canny edge on blurred hue", cannyblurhue)


    #Create another empty image variable.
    cannysat = numpy.zeros((cv_image.shape), numpy.uint8)
    #Fill the new image variable with a canny edge detection map of the greyscale image created earlier.
    cannysat = cv2.Canny(saturation, 50, 150, 3)
    #Display the canny mapping.
#    cv2.imshow("Canny Edge Detection satuartion", cannysat)	
		

    #3ms wait
    cv2.waitKey(3)


def ik_solver_request(input_limb, input_pose):
    print "IK solver request:"

    #input error checking
    if len(input_pose) == 6:
        quaternion_pose = conversions.list_to_pose_stamped(input_pose, "base")
    elif len(input_pose) == 7:
        quaternion_pose = input_pose
    else:
        print """Invalid Pose List:
    Input Pose to function: ik_solver_request must be a list of:
    6 elements for an RPY pose, or
    7 elements for a Quaternion pose"""
        return

    if input_limb == "right" or input_limb == "left":
        limb = input_limb
    else:
        print """Invalid Limb:
    Input Limb to function: ik_solver_request must be a string:
    'right' or 'left'"""
        return

    #request/response handling
    node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    ik_service = rospy.ServiceProxy(node, SolvePositionIK)
    ik_request = SolvePositionIKRequest()
    ik_request.pose_stamp.append(quaternion_pose)
    try:
        rospy.wait_for_service(node, 5.0)
        ik_response = ik_service(ik_request)
    except (rospy.ServiceException, rospy.ROSException), error_message:
        rospy.logerr("Service request failed: %r" % (error_message,))
    if (ik_response.isValid[0]):
        print("PASS: Valid joint configuration found")
        #convert response to JP control dict
        limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
        return limb_joints
    else:
        print("FAILED: No valid joint configuration for this pose found")





if __name__ == '__main__':
    rospy.init_node('activerobots_baxter_training_example', anonymous=True)

#create subscriber to the right hand camera, each frame recieved calls the callback function
camera_sub = rospy.Subscriber("/cameras/right_hand_camera/image",Image,callback)

#Reading Sensors
#read joint angles
#print baxter_interface.Limb('right').joint_angles()
#read button state
#print baxter_interface.digital_io.DigitalIO('right_lower_button').state
#read IR rangefinder
#print baxter_interface.analog_io.AnalogIO('right_hand_range').state()


#Writing to actuators
#define pi so we can use radians easily
pi = math.pi
#Enable the actuators
baxter_interface.RobotEnable().enable()



print "ended, now spinning, Ctrl-c to exit"
#prevents program from exiting, allowing subscribers and publishers to keep operating
#in our case that is the camera subscriber and the image processing callback function
rospy.spin()

