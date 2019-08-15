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

		#init_pose = (1.4,1.57,-1.57,0.6,-0.5)
		#self.init_pose = (0,1.57,-1.57,1.57,0)
		self.init_pose = (0,math.pi/4,math.pi/2,-math.pi/4,-0.3)
		#init_pose = (0,1.57,0.7,0.7,0.5)
		#hold_pose = (0,-2,2,-0.3,0)
		#self.hold_pose = (1.57, 2.1, -2.1, 1.57, 0.65)
		self.hold_pose = (0, -2.1, 2.1, math.pi/2, 0.2)

		#set_pose = (0,1.57,0.7,0.7,0.5)
		self.release_pose = (0,math.pi/4,math.pi/2,-math.pi/4,-0.3)

		self.joint1 = rospy.Publisher('/arm_shoulder_pan_joint/command',Float64)
		self.joint2 = rospy.Publisher('/arm_shoulder_lift_joint/command',Float64)
		self.joint3 = rospy.Publisher('/arm_elbow_flex_joint/command',Float64)
		self.joint4 = rospy.Publisher('/arm_wrist_flex_joint/command',Float64)
		self.joint5 = rospy.Publisher('/gripper_joint/command',Float64)#0.65:正好抓住 -0.5:张开最大
		
		self.released_pub = rospy.Publisher('/arm_done', String)
		# adding program above
		rospy.Subscriber("/nav2arm",String, self.callback_carry)
		rospy.loginfo("Subscribe to topic carry .....")
		rospy.sleep(2)
		######################
		# Define five poses
				
	def callback_carry (self,msg):
		if msg.data == "init":
			self.joint1.publish(self.init_pose[0])
			self.joint2.publish(self.init_pose[1])
			self.joint3.publish(self.init_pose[2])
			self.joint4.publish(self.init_pose[3])
			self.joint5.publish(self.init_pose[4])
			arm_msg = "init_done"
			self.released_pub.publish(arm_msg)
		if msg.data == "grasp":
			self.joint5.publish(self.hold_pose[4])
			rospy.sleep(4)
			self.joint2.publish(self.hold_pose[1])
			self.joint1.publish(self.hold_pose[0])
			self.joint3.publish(self.hold_pose[2])
			rospy.sleep(1)
			self.joint4.publish(self.hold_pose[3])
			arm_msg = "grasp_done"
			self.released_pub.publish(arm_msg)
		if msg.data == "release":
			self.joint1.publish(self.init_pose[0])
			self.joint2.publish(self.init_pose[1])
			self.joint3.publish(self.init_pose[2])
			self.joint4.publish(self.init_pose[3])
			rospy.sleep(4)
			self.joint5.publish(self.init_pose[4])
			arm_msg = "release_done"
			self.released_pub.publish(arm_msg)
		if msg.data == "finish":
			self.joint5.publish(self.hold_pose[4])
			self.joint2.publish(self.hold_pose[1])
			self.joint1.publish(self.hold_pose[0])
			self.joint3.publish(self.hold_pose[2])
			rospy.sleep(1)
			self.joint4.publish(self.hold_pose[3])
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

