#!/usr/bin/env python

"""
Battery monitor for the Segway RMP platform.

Author:  Chris Dunkers, Worcester Polytechnic Institute
Version: June 23, 2014
"""
from rmp_msgs.msg import RMPFeedback
from python_ethernet_rmp.system_defines import *
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import rospy
import math
import time
import os

class JointStateMonitor:

	def __init__(self):
		"""
		Initialize the subscriptions and publishers of the node.
		"""
		self.jointStatePub = rospy.Publisher('rmp_joint_states', JointState, queue_size = 'None')

		rospy.Subscriber("rmp_feedback", RMPFeedback, self.get_batt_state)
		
		"""
		Get the battery parameters
		"""
		self.has_two_wheels = rospy.get_param('~has_two_wheels',True)
		self.has_caster_wheel = rospy.get_param('~has_caster_wheel',True)
		self.link_left_front = rospy.get_param('~link_left_front','base_link_left_wheel_joint')
		self.link_right_front = rospy.get_param('~link_right_front','base_link_right_wheel_joint')
		self.link_left_rear = rospy.get_param('~link_left_rear','base_link_left_rear_wheel_joint')
		self.link_right_rear = rospy.get_param('~link_right_rear','base_link_right_rear_wheel_joint')
		self.link_caster = rospy.get_param('~link_caster','caster_plate_link_caster_link_joint')
		tire_diameter = rospy.get_param('/ethernet_rmp/my_tire_diameter',DEFAULT_TIRE_DIAMETER_M)
		self.circumference = math.pi*tire_diameter;
		self.prev_caster_pos = 0
		
	def get_batt_state(self, rmp):
		"""
		Read in the current RMP feedback and publish the pose
		:param rmp: the RMP feedback message
		"""
		rmp_items = rmp.sensor_items
		rmp_values = rmp.sensor_values
		
		joint_state = JointState()
		
		names = [self.link_left_front, self.link_right_front,
					self.link_left_rear, self.link_right_rear, self.link_caster]
		pos = [0,0,0,0,0]
		vel = [0,0,0,0,0]
		
		"""
		get the values for the feedback items needed
		"""
		for x in range(0, len(rmp_items)):
			if rmp_items[x] == 'left_front_pos_m':
				pos[0] = -((rmp_values[x]/self.circumference) % 1.0)*(2*math.pi)
			elif rmp_items[x] == 'right_front_pos_m':
				pos[1] = ((rmp_values[x]/self.circumference) % 1.0)*(2*math.pi)
			elif rmp_items[x] == 'left_rear_pos_m':
				pos[2] = -((rmp_values[x]/self.circumference) % 1.0)*(2*math.pi)
			elif rmp_items[x] == 'right_rear_pos_m':
				pos[3] = ((rmp_values[x]/self.circumference) % 1.0)*(2*math.pi)
			elif rmp_items[x] == 'left_front_vel_mps':
				vel[0] = -rmp_values[x]
			elif rmp_items[x] == 'right_front_vel_mps':
				vel[1] = rmp_values[x]
			elif rmp_items[x] == 'left_rear_vel_mps':
				vel[2] = -rmp_values[x]
			elif rmp_items[x] == 'right_rear_vel_mps':
				vel[3] = rmp_values[x]
				
		if self.has_two_wheels:
			num = 2
		else:
			num = 4
			
		for x in range(0,num):
			joint_state.name.append(names[x])
			joint_state.position.append(pos[x])
			joint_state.velocity.append(vel[x])
		
		"""
		calculate (very rough) caster wheel position
		"""
		if self.has_caster_wheel:
			left_vel = -vel[0]
			right_vel = vel[1]
			if abs(left_vel) < 0.01:
				left_vel = 0
			if abs(right_vel) < 0.01:
				right_vel = 0
			if left_vel != 0 or right_vel != 0:
				step = .04
				target = math.atan2(right_vel, left_vel) - math.pi/4
				if target > math.pi:
					target -= 2*math.pi
				if target < -math.pi:
					target += 2*math.pi
				pos[4] = self.prev_caster_pos
				#calculate direction to turn
				if abs(pos[4] - target) >= step:
					direction = 1
					if pos[4] > target:
						if abs(target - pos[4]) < (2*math.pi + target - pos[4]):
							direction = -1
					else:
						if (2*math.pi - (pos[4] - target)) < (pos[4] - target):
							direction = -1
					#turn to target
                       		 	pos[4] += direction*step
					while pos[4] <= -math.pi:
						pos[4] += 2*math.pi
					while pos[4] > math.pi:
						pos[4] -= 2*math.pi
				else:
					pos[4] = target
				self.prev_caster_pos = pos[4]
			else:
				pos[4] = self.prev_caster_pos

		joint_state.name.append(names[4])
		joint_state.position.append(pos[4])
		joint_state.velocity.append(vel[4])
		
		"""
		Publish the state of the wheels/joints
		"""
		joint_state.header.stamp = rospy.Time.now()
		self.jointStatePub.publish(joint_state)

if __name__ == "__main__":
	rospy.init_node("rmp_joint_state")
	jointState = JointStateMonitor()
	rospy.loginfo("RMP Joint State Started")
	rospy.spin()
