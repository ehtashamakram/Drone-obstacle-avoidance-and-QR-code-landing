#!/usr/bin/env python

import rospy
import actionlib
import tf
import roslib
import threading
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# the list of points to patrol
waypoints = [
    ['land', (8.000000, -5.000000, 0)],
    ['target', (3.64833, -0.591554, 0)]]

n = 0

class Patrol:

    def __init__(self):
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')

    def set_goal_to_point(self, point):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = point[0]
        goal.target_pose.pose.position.y = point[1]
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, point[2])
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()


if __name__ == '__main__':
	rospy.init_node('patrolling')
	cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	twist_msg = Twist()
	
	while n<=400:
        	print("calibrating")			
		#Provide the naccessary time to adjust hieght
    		twist_msg.angular.z = 2.0
		cmd_vel_pub.publish(twist_msg)
        	rospy.sleep(0.1)
		print n
        	n+=1
	print("callibrated")
	try:
		p = Patrol()
		map_name = []
		for i, w in enumerate(waypoints):
			map_name.append(w[1])
	
		while not rospy.is_shutdown():
			request = str(raw_input("Type something to test this out: "))
			if request ==  "land":
				p.set_goal_to_point(map_name[0])
			if request ==  "target":
				p.set_goal_to_point(map_name[1])
	except rospy.ROSInterruptException:
		rospy.logerr("Something went wrong when sending the goal")
