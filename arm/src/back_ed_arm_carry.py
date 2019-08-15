#!/usr/bin/env python
# -*- coding: utf-8 -*-

# new calculating algorithm
import rospy
import math
from std_msgs.msg import UInt32
from std_msgs.msg import Float64
from std_msgs.msg import String
class Loop:
	def __init__(self):
		rospy.on_shutdown(self.cleanup)

		self.released_pub = rospy.Publisher('/arm_done', String)

		# the variable for diandong tuigan
		# self.mark = 0
		

		# adding program above
		rospy.Subscriber("/nav2arm",String, self.callback_carry)
		rospy.loginfo("Subscribe to topic carry .....")
		rospy.sleep(2)
		######################
		# Define five poses
				
	def callback_carry (self,msg):
    
		if msg.data == "init":
			rospy.sleep(5)
			arm_msg = "init_done"
			self.released_pub.publish(arm_msg)
		if msg.data == "grasp":
			rospy.sleep(5)
			arm_msg = "grasp_done"
			self.released_pub.publish(arm_msg)
		if msg.data == "release":
			rospy.sleep(5)
			arm_msg = "release_done"
			self.released_pub.publish(arm_msg)
		if msg.data == "finish":
			rospy.sleep(5)
			arm_msg = "finish_done"
			self.released_pub.publish(arm_msg)


	def cleanup(self):
		rospy.loginfo("Shutting down turtlebot arm....")

if __name__=="__main__":

	rospy.init_node('arm')
	try:
		Loop()
		rospy.spin()
	except rospy.ROSInterruptException:
			pass

