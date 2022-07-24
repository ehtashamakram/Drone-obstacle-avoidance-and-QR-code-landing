#!/usr/bin/env python

import numpy as np
import rospy, cv2, cv_bridge
import sys, time, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from std_msgs.msg import String
import numpy

#--- Define Tag
height=0
distance = 1.0
upword = 0
height_speed = 0.3				#How fast the drone go up
desired_height=1.2
downward = 0
take_off = "true"

def height_callback(msg):
    global height, take_off
    height = msg.range
    return height



def main(msg):
    global cmd_vel_pub, distance, upword, downward, pub, take_off
    global height

    #--- Get the camera calibration path
    twist = Twist()


    #--- Capture the videocamera (this may also be a video or a picture)
    cap = cv_bridge.CvBridge()

    #-- Font for the text in the image
    font = cv2.FONT_HERSHEY_PLAIN

    #while True:

    #-- Read the camera frame
    frame = cap.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    detector = cv2.QRCodeDetector()
    data, bbox, _ = detector.detectAndDecode(frame)

    request = "True"
    #if bbox is None:
    #	twist.linear.z = 0.5
    #	cmd_vel_pub.publish(twist)
			

    
    if bbox is not None:
			take_off == "false"
			pub.publish(request)
			print " Landing Pad Found"
			marks = cap.imgmsg_to_cv2(msg,desired_encoding='bgr8')
			marker = cv2.rectangle(marks,(bbox[1][0][0],bbox[1][0][1]),(bbox[3][0][0],bbox[3][0][1]),(45,255,90),-1)
		
			hsv = cv2.cvtColor(marker, cv2.COLOR_BGR2HSV)
			LowerYellow = numpy.array([ 29, 86, 6])
			UpperYellow = numpy.array([64, 255, 255])
			mask = cv2.inRange(hsv, LowerYellow, UpperYellow)
	
			# Calculating the the mask coordinates on marker
			h, w, d = marker.shape
			Top_Search = 3
			bottom_Search = Top_Search + 300
			mask[0:Top_Search, 0:w] = 0
			mask[bottom_Search:h, 0:w] = 0
			M = cv2.moments(mask)
	
			if M['m00'] > 0:
				dx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				cv2.circle(marker, (dx, cy), 1, (0,0,255), -1)
		
				#Calculating the error value that how much motors has to rotate

				err_x = dx - w/2
				err_y = cy - h/2
				twist.linear.x = -float(err_y) / 95
				twist.angular.z = -float(err_x) / 95
				twist.linear.z = -0.2
				if height <= 0.6:
					downward = 1
			cmd_vel_pub.publish(twist)
    
    if downward == 1:
	twist.angular.z = 0
	twist.linear.x = 0
	twist.linear.z = -0.5
	print "landing"
	cmd_vel_pub.publish(twist)
						
    #--- Display the frame
    cv2.imshow('bottom_camera', frame)

    #--- use 'q' to quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cv2.destroyAllWindows()

def land():
	global cmd_vel_pub, pub
	print "finding landing pad"
	height_sub = rospy.Subscriber('/sonar_height', Range, height_callback)
	image_sub = rospy.Subscriber('/bottom/camera/image', Image, main)
	pub = rospy.Publisher('land', String, queue_size=1)
	cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

	

if __name__ == '__main__':
		cv2.destroyAllWindows()
		rospy.init_node('landing')
		land()
		rospy.spin()
