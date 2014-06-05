#!/usr/bin/env python
import roslib 
roslib.load_manifest('segway_rmp')
import rospy

"""--------------------------------------------------------------------
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
--------------------------------------------------------------------"""

from ros_ethernet_rmp.msg import rmpCommand, rmpFeedback
from python_ethernet_rmp.rmp_interface import RMP
from python_ethernet_rmp.system_defines import *
from python_ethernet_rmp.user_event_handlers import RMPEventHandlers
from python_ethernet_rmp.rmp_config_params import *
import sys,time,threading,Queue

"""
Define the update delay or update period in seconds. Must be greater
than the minimum of 0.01s
"""
UPDATE_DELAY_SEC = 0.02

"""
Define whether to log the output data in a file. This will create a unique CSV
file in ./RMP_DATA_LOGS with the filename containing a time/date stamp 
"""
LOG_DATA = True			 

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
		params = []
		params.append(['my_velocity_limit_mps',rospy.get_param('~my_velocity_limit_mps ',0.5)])
		params.append(['my_accel_limit_mps2',rospy.get_param('~my_accel_limit_mps2 ',0.981)])
		params.append(['my_decel_limit_mps2',rospy.get_param('~my_decel_limit_mps2 ',0.981)])
		params.append(['my_coastdown_accel_mps2',rospy.get_param('~my_coastdown_accel_mps2 ',0.1962)])
		params.append(['my_yaw_rate_limit_rps',rospy.get_param('~my_yaw_rate_limit_rps ',0.5)])
		params.append(['my_yaw_accel_limit_rps2',rospy.get_param('~my_yaw_accel_limit_rps2 ',0.5)])
		params.append(['my_tire_diameter_m',rospy.get_param('~my_tire_diameter_m ',I2_TIRE_DIAMETER_M)])
		params.append(['my_wheel_base_length_m',rospy.get_param('~my_wheel_base_length_m ',DEFAULT_WHEEL_BASE_LENGTH_M)])
		params.append(['my_wheel_track_width_m',rospy.get_param('~my_wheel_track_width_m ',I2_WHEEL_TRACK_WIDTH_M)])
		params.append(['my_gear_ratio',rospy.get_param('~my_gear_ratio ',I2_TRANSMISSION_RATIO)])
		params.append(['my_config_bitmap',rospy.get_param('~my_config_bitmap ',1)])
		params.append(['my_ip_address',rospy.get_param('~my_ip_address ',DEFAULT_IP_ADDRESS)])
		params.append(['my_port_num',rospy.get_param('~my_port_num ',DEFAULT_PORT_NUMBER)])
		params.append(['my_subnet_mask',rospy.get_param('~my_subnet_mask ',DEFAULT_SUBNET_MASK)])
		params.append(['my_gateway',rospy.get_param('~my_gateway ',DEFAULT_GATEWAY)])
		params.append(['my_user_defined_feedback_bitmap_1',rospy.get_param('~my_user_defined_feedback_bitmap_1 ',DEFAULT_USER_FB_1_BITMAP)])
		params.append(['my_user_defined_feedback_bitmap_2',rospy.get_param('~my_user_defined_feedback_bitmap_2 ',DEFAULT_USER_FB_2_BITMAP)])
		params.append(['my_user_defined_feedback_bitmap_3',rospy.get_param('~my_user_defined_feedback_bitmap_3 ',DEFAULT_USER_FB_3_BITMAP)])
		params.append(['my_user_defined_feedback_bitmap_4',rospy.get_param('~my_user_defined_feedback_bitmap_4 ',DEFAULT_USER_FB_4_BITMAP)])
		
		SetRMPConfigParams(params)
				
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
		self.my_thread = threading.Thread(target=RMP, args=(rmp_addr,self.rsp_queue,self.cmd_queue,self.in_flags,self.out_flags,UPDATE_DELAY_SEC,LOG_DATA))
		self.my_thread.daemon = True
		self.my_thread.start()
		
		"""
		Initialize my event handler class
		"""
		self.EventHandler = RMPEventHandlers(self.cmd_queue,self.rsp_queue,self.in_flags,self)
		
		"""
		Initialize the feedback publisher
		"""
		self.rmpFeedback = rmpFeedback()
		self.feedbackPub = rospy.Publisher('rmp_feedback', rmpFeedback)
		
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
		print 'exited rmp_exchagne'
				
	def sendCommand(self,command):
		#make into RMP command format
		self.EventHandler.AddCommand(command)
		
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
			
	def rmp_send_recv(self):
		"""
		Initialize the ROS node
		"""
		rospy.init_node('rmp_exchange')
		rospy.Subscriber("rmp_command", rmpCommand, self.sendCommand)
		print "RMP exchange node started."
		
		self.EventHandler.AddListener(self.publishFeedback)
		
		"""
		Generate a goto tractor event
		"""
		self.EventHandler.GotoTractor()
		print "In Tractor Mode"
		while not rospy.is_shutdown() and self.EventHandler._continue:
			while not self.out_flags.empty() and self.EventHandler._continue:
				self.EventHandler.handle_event[self.out_flags.get()]()
			
if __name__ == "__main__":
	rmp_command = RMPExchange()
	rmp_command.rmp_send_recv()	

