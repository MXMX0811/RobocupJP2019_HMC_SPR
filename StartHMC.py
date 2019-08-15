#!/usr/bin/env python
# -*- coding:utf-8 -*-
import os

os.system('gnome-terminal -x bash -c "roslaunch turtlebot_bringup minimal.launch"')
os.system('sleep 5')
#os.system('gnome-terminal -x bash -c "roslaunch kamerider_image_detection double_kinect.launch"')
#os.system('gnome-terminal -x bash -c "roslaunch freenect_launch freenect.launch"')
#os.system('sleep 8')
os.system('gnome-terminal -x bash -c "roslaunch kamerider_navigation hmc_navigation.launch"')
os.system('sleep 5')
os.system('gnome-terminal -x bash -c "roslaunch kamerider_speech hmc_follower.launch"')
os.system('sleep 5')
os.system('gnome-terminal -x bash -c "roslaunch kamerider_speech speech_all_hmc.launch"')
os.system('sleep 2')
os.system('gnome-terminal -x bash -c "roslaunch arm ed_arm_carry.launch"')
