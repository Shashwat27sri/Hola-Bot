#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (KB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			HB_3347
# Author List:		Ajay Kumar Sahu, Shashwat Srivastava, Adiljith Babu, Kunal Sagar Prasad Singh
# Filename:			task_0.py
# Functions:
# 					[callback, main]
# Nodes:			Add your publishing and subscribing node


####################### IMPORT MODULES #######################
import sys
import traceback
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import *
##############################################################


def callback(data):
	global pose
	pose = data
	pose.x = pose.x
	pose.y = pose.y
	pose.theta = pose.theta


def main():
	global pose, init_pose, stages
	rospy.init_node('turtlebot_controller', anonymous=True)
	velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
	sub = rospy.Subscriber('/turtle1/pose',Pose, callback)
	
	rate = rospy.Rate(60)
	vel_msg = Twist()

	# setInitPose()
	while init_pose.x == 0 and init_pose.y ==0:
		# print(pose.x,init_pose.x)
		velocity_publisher.publish(vel_msg)
		rate.sleep()
		init_pose.x = pose.x
		init_pose.y = pose.y
		init_pose.theta = pose.theta

	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0

	# Angular velocity in the z-axis.
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0

	v = 0.5

	
	while not rospy.is_shutdown():
		# print(pose.theta)
		if stages[0] == 0:
			if pose.theta >=0 and pose.theta < pi :
				vel_msg.linear.x = v
				vel_msg.angular.z = v
			else:
				vel_msg.linear.x = 0
				vel_msg.angular.z = v
				stages[0] = 1
		
		elif stages[1] == 0:
			if pose.theta >= -pi/2 :
				vel_msg.angular.z = 0
				vel_msg.linear.x = v
				stages[1] = 1

		elif stages[2] == 0:
			if pose.y > init_pose.y:
				vel_msg.linear.x = v
			else:
				vel_msg.linear.x = 0
				stages[2] == 1


		velocity_publisher.publish(vel_msg)
		rate.sleep()
	




################# ADD GLOBAL VARIABLES HERE #################

pose = Pose()
init_pose = Pose()
stages = [0,0,0]

##############################################################


################# ADD UTILITY FUNCTIONS HERE #################



##############################################################


######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS PART #########
if __name__ == "__main__":
	try:
		print("------------------------------------------")
		print("         Python Script Started!!          ")
		print("------------------------------------------")
		main()

	except:
		print("------------------------------------------")
		traceback.print_exc(file=sys.stdout)
		print("------------------------------------------")
		sys.exit()

	finally:
		print("------------------------------------------")
		print("    Python Script Executed Successfully   ")
		print("------------------------------------------")