#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Empty
from move_base_msgs.msg import MoveBaseActionFeedback
from std_msgs.msg import String

DRONE_HEIGHT = 0.3
isGoal = False
land = False
n = 0


def TakeoffLand():
    global landing, isGoal,twist_msg, cmd_vel_pub, twist_active
    rospy.init_node('takeoff_land') # Creates the node

    # Instance Variables
    landing = False # If quadcopter in landing mode or not- initially set to false unless otherwise
    isGoal = False # If quadcopter has a current goal to reach - initially set to false unless otherwise
    twist_active = True # If quadcopter is currenty being given a command velocity - initially set to true unless otherwise
    twist_msg = Twist() # Twist message to publish for hovering

    # Subcribers
    land_sub = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, goal_cb) # Information about move_base goal
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_cb) # Allows access for current command velocities published
    takeoff_sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, takeoff_cb) # Information about pose of the drone
    land_sub = rospy.Subscriber('/quadcopter_land', Empty, land_cb) # Listens for a command to land drone
    sub = rospy.Subscriber('land', String, callback)

    # Publishers
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # For sending z command velocity for takeoff and hover
   
    rate = rospy.Rate(5) # rate at which to revisit callbacks
    rospy.spin() # keeps the node alive



def takeoff_cb(msg):

    global landing, isGoal, twist_msg, cmd_vel_pub, land
    z_pos = msg.pose.position.z # the current z position of the drone

    # If the drone is not landing or if the drone is not currently completing a 2D Nav goal
    if landing == False and isGoal == False and land == False:

      # If the current x and y velocities are 0
      if twist_msg.linear.x == 0 and twist_msg.linear.y == 0:
        # if the z_pos is between 1 and 1.3 meters, set a small z vel
        if z_pos > 1 and z_pos < 1.3:

          print ("Hieght = " +str(msg.pose.position.z))
          twist_msg.linear.z = 0.1

        # If the z_pos is between 1.3 and 1.5 meters, hover
        elif z_pos > 1.3 and z_pos < 1.5:
          print ("Hieght = " +str(msg.pose.position.z))
          twist_msg.linear.z = 0.0
        
        # If the z_pos is between 1.5 and 1.8 meters, set a small neg z vel
        elif z_pos > 1.5 and z_pos < 1.8:
          print ("Hieght = " +str(msg.pose.position.z))
          twist_msg.linear.z = -0.1

        # If the z_pos is greater than 1.8 meters, set a med neg z vel
        elif z_pos > 1.8:
          print ("Hieght = " +str(msg.pose.position.z))
          twist_msg.linear.z = -0.3

        # If the drone is below 1 meter, takeoff with pos z vel
        else:
          print ("Hieght = " +str(msg.pose.position.z))
          twist_msg.linear.z = 0.3
      
        # Publish z velocity
        cmd_vel_pub.publish(twist_msg)

    # OR if the drone is completing a goal
    elif isGoal == True:

      # If a command velocity is not actually being set for the drone
      if twist_active == False:
        # The drone is actually not completing a goal
        isGoal = False

      # If a command velocity is being set for the drone
      else:
        print ("Moving towards Goal")

    # OR if the drone is currently landing
    elif landing == True:

      global DRONE_HEIGHT
      # If the drone is above the ground
      if z_pos >= DRONE_HEIGHT:
        # Set a negative z velocity for the drone to come down and publish
        twist_msg.linear.z = -0.3
        cmd_vel_pub.publish(twist_msg)
        print ("LANDING")




      # Once drone has reached ground, shut off takeoff_land node
      else:
        rospy.loginfo("System is shutting down. Stopping drone...")
        rospy.signal_shutdown("DRONE HAS LANDED")

    elif land == True:
	print ("LANDING")
	


def land_cb(msg):
    global landing

    landing = True

def goal_cb( msg):
    global landing, isGoal

    isGoal = True

def cmd_vel_cb(msg):
    global twist_active
    # If all values in the cmd_vel message are 0, which means the drone is temporarily not moving
    if msg.linear.x == 0 and msg.linear.y == 0 and msg.linear.z == 0 and msg.angular.x == 0 and msg.angular.y == 0 and msg.angular.z == 0:
      # The command velocities are not active
      twist_active = False
    else:
      # The command velocities are active
      twist_active = True

def callback(msg):
    global land
    land = True



                                                         

if __name__ == '__main__':
    TakeoffLand()
