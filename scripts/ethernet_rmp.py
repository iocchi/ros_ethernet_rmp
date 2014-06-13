#!/usr/bin/env python

"""
--------------------------------------------------------------------
COPYRIGHT 2013 SEGWAY Inc.

Software License Agreement:

The software supplied herewith by Segway Inc. (the "Company") for its
RMP Robotic Platforms is intended and supplied to you, the Company's
customer, for use solely and exclusively with Segway products. The
software is owned by the Company and/or its supplier, and is protected
under applicable copyright laws.  All rights are reserved. Any use in
violation of the foregoing restrictions may subject the user to criminal
sanctions under applicable laws, as well as to civil liability for the
breach of the terms and conditions of this license. The Company may
immediately terminate this Agreement upon your use of the software with
any products that are not Segway products.

The software was written using Python programming language.  Your use
of the software is therefore subject to the terms and conditions of the
OSI- approved open source license viewable at http://www.python.org/.
You are solely responsible for ensuring your compliance with the Python
open source license.

You shall indemnify, defend and hold the Company harmless from any claims,
demands, liabilities or expenses, including reasonable attorneys fees, incurred
by the Company as a result of any claim or proceeding against the Company
arising out of or based upon:

(i) The combination, operation or use of the software by you with any hardware,
	products, programs or data not supplied or approved in writing by the Company,
	if such claim or proceeding would have been avoided but for such combination,
	operation or use.

(ii) The modification of the software by or on behalf of you

(iii) Your use of the software.

THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
--------------------------------------------------------------------
"""

"""
ROS wrapper for the python ethernet RMP driver. This is an adaptation of example code provided by Segway Inc.

Author:  Segway Inc.
Author:  Chris Dunkers, Worcester Polytechnic Institute
Author:  Russell Toris, Worcester Polytechnic Institute
Version: June 10, 2014
"""

from ros_ethernet_rmp.msg import RMPCommand, RMPFeedback
from python_ethernet_rmp.rmp_interface import RMP
from python_ethernet_rmp.system_defines import *
from python_ethernet_rmp.user_event_handlers import RMPEventHandlers
from python_ethernet_rmp.rmp_config_params import *
from python_ethernet_rmp.utils import *
from geometry_msgs.msg import Twist
from rmp_config_limits import *
import sys,time,threading,Queue
import math
import rospy

"""
The platform address may be different than the one in your config
(rmp_config_params.py). This would be the case if you wanted to update 
ethernet configuration. If the ethernet configuration is updated the
system needs to be power cycled for it to take effect and this should
be changed to match the new values you defined in your config
"""
rmp_addr = ("192.168.0.40",8080) #this is the default value and matches the config

"""
Define the main function for the example. It creates a thread to run RMP and handles
passing the events to the user defined handlers in user_event_handlers.py
"""

class RMPExchange:
	def __init__(self):
		"""
		Initialze the thread 
		"""
				
		"""
		Read in the Ros Params and add them to a array to set the config params
		"""
		global rmp_addr
		update_delay_sec = rospy.get_param('~update_delay_sec', 0.05)
		log_data = rospy.get_param('~log_data', False)
		ip_addr = rospy.get_param('~current_rmp_ip_addr', DEFAULT_IP_ADDRESS)
		port_num = rospy.get_param('~current_rmp_port_num', DEFAULT_PORT_NUMBER)
		self.isOmni = rospy.get_param('~is_omni ',False)
		try:
			dottedQuadToNum(ip_addr)
			if port_num > 0:
				rmp_addr = (ip_addr,port_num)
			else:
				rospy.logwarn("current_rmp_port_num is not a valid port number")	
		except:
			rospy.logwarn("current_rmp_ip_addr in not in dotted quad format")

		if update_delay_sec < 0.01:
			rospy.logwarn("Update delay time is too fast -- setting to 0.01 seconds.")
			update_delay_sec = 0.01

		self.rmpParams = []
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_MAXIMUM_VELOCITY,rospy.get_param('~my_velocity_limit_mps',DEFAULT_MAXIMUM_VELOCITY_MPS)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_MAXIMUM_ACCELERATION,rospy.get_param('~my_accel_limit_mps2',DEFAULT_MAXIMUM_ACCELERATION_MPS2)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_MAXIMUM_DECELERATION,rospy.get_param('~my_decel_limit_mps2',DEFAULT_MAXIMUM_DECELERATION_MPS2)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_MAXIMUM_DTZ_DECEL_RATE,rospy.get_param('~my_dtz_rate_mps2',DEFAULT_MAXIMUM_DTZ_DECEL_RATE_MPS2)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_COASTDOWN_ACCEL,rospy.get_param('~my_coastdown_accel_mps2',DEFAULT_COASTDOWN_ACCEL_MPS2)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_MAXIMUM_TURN_RATE,rospy.get_param('~my_yaw_rate_limit_rps',DEFAULT_MAXIMUM_YAW_RATE_RPS)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_MAXIMUM_TURN_ACCEL,rospy.get_param('~my_yaw_accel_limit_rps2',DEFAULT_MAX_YAW_ACCEL_RPS2)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_TIRE_DIAMETER,rospy.get_param('~my_tire_diameter_m',DEFAULT_TIRE_DIAMETER_M)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_WHEEL_BASE_LENGTH,rospy.get_param('~my_wheel_base_length_m',DEFAULT_WHEEL_BASE_LENGTH_M)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_WHEEL_TRACK_WIDTH,rospy.get_param('~my_wheel_track_width_m',DEFAULT_WHEEL_TRACK_WIDTH_M)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_TRANSMISSION_RATIO,rospy.get_param('~my_gear_ratio',DEFAULT_TRANSMISSION_RATIO)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_INPUT_CONFIG_BITMAP,rospy.get_param('~my_config_bitmap',DEFAULT_CONFIG_BITMAP)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_ETH_IP_ADDRESS,rospy.get_param('~my_ip_address',DEFAULT_IP_ADDRESS)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_ETH_PORT_NUMBER,rospy.get_param('~my_port_num',DEFAULT_PORT_NUMBER)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_ETH_SUBNET_MASK,rospy.get_param('~my_subnet_mask',DEFAULT_SUBNET_MASK)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_ETH_GATEWAY,rospy.get_param('~my_gateway',DEFAULT_GATEWAY)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_USER_FB_1_BITMAP,rospy.get_param('~my_user_defined_feedback_bitmap_1',DEFAULT_USER_FB_1_BITMAP)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_USER_FB_2_BITMAP,rospy.get_param('~my_user_defined_feedback_bitmap_2',DEFAULT_USER_FB_2_BITMAP)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_USER_FB_3_BITMAP,rospy.get_param('~my_user_defined_feedback_bitmap_3',DEFAULT_USER_FB_3_BITMAP)])
		self.rmpParams.append([RMP_CFG_CMD_ID,RMP_CMD_SET_USER_FB_4_BITMAP,rospy.get_param('~my_user_defined_feedback_bitmap_4',DEFAULT_USER_FB_4_BITMAP)])
				
		"""
		Create and response and command queue. The responses will be in the form of 
		a dictionary containing the vaiable name as the key and a converted value
		the names are defined in the feedback_X_bitmap_menu_items dictionaries if a particular
		variable is of interest
		"""
		self.rsp_queue = Queue.Queue()
		self.cmd_queue = Queue.Queue()
		self.in_flags  = Queue.Queue()
		self.out_flags = Queue.Queue()
		
		"""
		Create the thread to run RMP 
		"""
		self.my_thread = threading.Thread(target=RMP, args=(rmp_addr,self.rsp_queue,self.cmd_queue,self.in_flags,self.out_flags,update_delay_sec, log_data))
		self.my_thread.daemon = True
		self.my_thread.start()
		
		"""
		Initialize my event handler class
		"""
		self.EventHandler = RMPEventHandlers(self.cmd_queue,self.rsp_queue,self.in_flags,self)
		
		"""
		Initialize the feedback publisher
		"""
		self.rmpFeedback = RMPFeedback()
		self.feedbackPub = rospy.Publisher('rmp_feedback', RMPFeedback, queue_size = 'None')

		rospy.Subscriber("rmp_command", RMPCommand, self.sendRMPCommand)
		rospy.Subscriber("cmd_vel", Twist, self.sendMotionCommand)

		"""
		Add a Feedback listener to the segway driver
		"""
		self.EventHandler.AddListener(self.publishFeedback)

		"""
		Initialize the parameters to the Segway
		"""
		self.initRMPParams()

		"""
		Generate a goto tractor event
		"""
		self.EventHandler.GotoTractor()
		
	def __del__(self):
		"""
		send the signal to kill the thread
		"""
		self.in_flags.put(RMP_KILL)
		
		"""
		Wait for the thread to die
		"""
		while self.my_thread.isAlive():
			pass
		
		"""
		Exit main
		"""
		#sys.exit()
		print 'exited rmp_exchange'
				
	def sendMotionCommand(self,command):
		if not self.isOmni:
			self.EventHandler.AddCommand([RMP_MOTION_CMD_ID,command.linear.x,-command.angular.z])
		else:
			forw_vel = math.sqrt(command.linear.x**2 + command.linear.y**2)/math.sqrt(2)
			angle = math.atan2(command.linear.y, command.linear.x) + math.pi
			self.EventHandler.AddCommand([RMP_OMNI_MOTION_CMD_ID,forw_vel,-command.angular.z, angle])
	
	def sendRMPCommand(self,command):
		if command.cmd_id == 1280 or command.cmd_id == 1281: 
			cmd = [command.cmd_id, command.arg1, command.arg2] 
			if self.isValidCommand(cmd):
				self.EventHandler.AddCommand(cmd)
		elif command.cmd_id == 1536:
			cmd = [command.cmd_id, command.arg1, command.arg2, command.arg3] 
			if self.isValidCommand(cmd):
				self.EventHandler.AddCommand(cmd)
		
	def publishFeedback(self,fb_dict):
		snrValues = []
		snrItems = []
		fltValues = []
		fltItems = []
		ipValues = []
		ipItems = []
		
		"""
		get all of the values from the Feedback provided by the RMP
		"""
		for key, value in fb_dict.items():
			try:
				value = float(value)
				snrValues.append(value)
				snrItems.append(key)
			except:
				try:
					value = int(value,16)
					fltValues.append(value)
					fltItems.append(key)
				except:
					ipValues.append(value)
					ipItems.append(key)
					
		self.rmpFeedback.header.stamp = rospy.Time.now()
		self.rmpFeedback.header.frame_id = 'base_link'
		self.rmpFeedback.sensor_items = snrItems
		self.rmpFeedback.sensor_values = snrValues
		self.rmpFeedback.fault_status_items = fltItems
		self.rmpFeedback.fault_status_values = fltValues
		self.rmpFeedback.ip_info = ipItems
		self.rmpFeedback.ip_values = ipValues
		self.feedbackPub.publish(self.rmpFeedback)
			
	def isValidCommand(self, command):
		if command[0] == 1280:
			return [True,command]
		elif command[0] == 1536:
			if command[3] > 360 or command[3] < 0:
				rospy.logwarn("arg3 is out of range. Must be between 0-360")
				return [False,command]
			else:
				return [True,command]
		elif command[0] == 1281:
			param_func = config_params_function.setdefault(command[1],is_not_param)
			checks = param_func(command[1],command[2])
			if checks[1] == True:
				command[2] = checks[0]
				return [True,command]
			else:
				return [False,command]
		else:
			rospy.logwarn("cmd_id is invalid")
			return [False,command]
	
	def initRMPParams(self):
		for command in self.rmpParams:
			check = self.isValidCommand(command)
			if check[0] == True:
				self.EventHandler.AddCommand(check[1])
	
	def rmp_send_recv(self):
		"""
		Main loop to continually empty yhe out_flags queue
		"""
		while not rospy.is_shutdown() and self.EventHandler._continue:
			while not self.out_flags.empty() and self.EventHandler._continue:
				self.EventHandler.handle_event[self.out_flags.get()]()
			
if __name__ == "__main__":
	rospy.init_node('ros_ethernet_rmp')
	rmp_command = RMPExchange()
	rospy.loginfo("ROS Ethernet RMP Node Started")
	rmp_command.rmp_send_recv()	
