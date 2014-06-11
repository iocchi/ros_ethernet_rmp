#!/usr/bin/env python

"""
Pose updater for the Segway RMP platform.

Author:  Chris Dunkers, Worcester Polytechnic Institute
Author:  Russell Toris, Worcester Polytechnic Institute
Version: June 10, 2014
"""
from ros_ethernet_rmp.msg import RMPFeedback
from tf.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import *
import tf
import rospy
import math
import time


class PoseUpdate:
	"""
	Pose updater for the Segway RMP platform.
	"""

	prev_lin_pos = 0.0
	prev_yaw = 0.0
	prev_x_pos = 0.0
	prev_y_pos = 0.0
	prev_time = 0.0

	def __init__(self):
		"""
		Initialize the subscriptions and publishers of the node.
		"""
		self.odomPub = rospy.Publisher('odom', Odometry, queue_size = 'None')
		self.tfBroadCast = tf.TransformBroadcaster()
		rospy.Subscriber("rmp_feedback", RMPFeedback, self.pose_update)
		
	def pose_update(self, rmp):
		"""
		Read in the current RMP feedback and publish the pose
		:param rmp: the RMP feedback message
		"""
		rmp_items = rmp.sensor_items
		rmp_values = rmp.sensor_values
		time_msg_received = rmp.header.stamp.secs + 10**-9 * rmp.header.stamp.nsecs
		
		"""
		get the values for the feedback items needed
		
		angles and angle rates are flipped because the RMP's do not 
		follow the right hand rule convention
		"""
		for x in range(0, len(rmp_items)):
			if rmp_items[x] == 'linear_vel_mps':
				lin_vel = rmp_values[x]
			elif rmp_items[x] == 'linear_pos_m':
				lin_pos = rmp_values[x] 
			elif rmp_items[x] == 'differential_wheel_vel_rps':
				diff_rate = -1 * rmp_values[x]
			elif rmp_items[x] == 'angle_target_deg':
				ang_targ = (rmp_values[x] - 90) * math.pi/180
			elif rmp_items[x] == 'pse_pitch_deg':
				pitch = (-1 * rmp_values[x]) * math.pi/180
			elif rmp_items[x] == 'pse_roll_deg':
				roll = (-1 * rmp_values[x]) * math.pi/180
		
		"""
		Segway RMP base tends to drift when stationary, basic 
		filter to ignore low differences and reduce/stop drift
		"""	
		if diff_rate >= -0.005 and diff_rate <= 0.005:
			diff_rate = 0.0

		"""
		Calculate the new pose based on the feedback from the Segway
		and the time difference from the previous calculation
		"""
		yaw = self.prev_yaw + diff_rate * (time_msg_received - self.prev_time)
		x_pos = self.prev_x_pos + (lin_pos - self.prev_lin_pos) * math.cos(yaw)
		y_pos = self.prev_y_pos + (lin_pos - self.prev_lin_pos) * math.sin(yaw)

		# store the current values to be used in the next iteration
		self.prev_lin_pos = lin_pos
		self.prev_x_pos = x_pos
		self.prev_y_pos = y_pos
		self.prev_yaw = yaw
		self.prev_time = time_msg_received

		# create quaternion array from rmp and IMU data
		quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

		# make and publish the odometry message
		odom = Odometry()
		odom.header.stamp = rospy.Time.now()
		odom.header.frame_id = "odom"
		odom.child_frame_id = "base_link"
		odom.pose.pose.position.x = x_pos
		odom.pose.pose.position.y = y_pos
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation.x = quaternion[0]
		odom.pose.pose.orientation.y = quaternion[1]
		odom.pose.pose.orientation.z = quaternion[2]
		odom.pose.pose.orientation.w = quaternion[3]
		odom.twist.twist.linear.x = lin_vel * math.cos(ang_targ)
		odom.twist.twist.linear.y = lin_vel * math.sin(ang_targ)
		odom.twist.twist.angular.z = diff_rate
		self.odomPub.publish(odom)

		# publish the transform from odom to the base footprint
		self.tfBroadCast.sendTransform((x_pos, y_pos, 0.0), quaternion, rospy.Time.now(), '/base_link', '/odom')

if __name__ == "__main__":
	rospy.init_node("rmp_pose_updater")
	poseUpdate = PoseUpdate()
	rospy.loginfo("RMP Pose Updater Started")
	rospy.spin()
