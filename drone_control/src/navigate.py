#!/usr/bin/env python
import rospy, cv2, cv_bridge
import actionlib
import tf
import roslib
import threading
import sys, time, math
import numpy as np
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

get_pos = 0

def callback(msg):
    global x, y, z,h
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    h = msg.pose.pose.position.z
    z = msg.pose.pose.orientation.z

if get_pos == 0:
	odo_sub = rospy.Subscriber('/ground_truth/state', Odometry, callback)
	get_pos = 1

# Get a move_base action client
rospy.init_node('patrolling')
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
rospy.loginfo('Connecting to move_base...')
client.wait_for_server()
rospy.loginfo('Connected to move_base.')

def set_goal_to_point(point):

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = point[0]
    goal.target_pose.pose.position.y = point[1]
    goal.target_pose.pose.position.y = point[2]
    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, point[3])
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


marker_size  = 50 #- [cm]
stopp = 0
add = -20 
yaw = 3.14
n = 0
m = 0
a = 0
b = 0
c = 0

def main():
    global x, y, z, stopp, n, m, add, yaw, a,b,c
    print"start point"
    point = (x+1,y+2.5,h+1,z+yaw)
    print "goal = " + str(point)
    set_goal_to_point(point)

main()

