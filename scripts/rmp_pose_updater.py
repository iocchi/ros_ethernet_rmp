#!/usr/bin/env python
import roslib;
roslib.load_manifest('segway_pose')

from ros_ethernet_rmp.msg import rmpFeedback
from tf.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import *
import tf
import rospy
import math
import time

class PoseUpdate:

	initYaw = 0
	initXPos = 0
	initYPos = 0
	linVel = 0
	linPos = 0
	yawRate = 0
	firstUpdate = 1
	prevLinPos = 0
	prevYaw = 0
	prevTime = 0
	timeMsgReceived = 0
	updateVar = 0;

	def __init__(self):
		self.odomPub = rospy.Publisher('odom', Odometry)
		self.tfBroadCast = tf.TransformBroadcaster()
		
	def pose_update(self,rmp):
		
		"""
		Get the necessary feedback items
		"""
		rmpItems = rmp.sensor_items
		rmpVals = rmp.sensor_values
		self.timeMsgReceived = rmp.header.stamp.secs

		"""
		Get the values for the feedback items needed
		"""
		for x in range(0, len(rmpItems)):
			if rmpItems[x] == 'linear_vel_mps':
				self.linVel = rmpVals[x]
			elif rmpItems[x] == 'linear_pos_m':
				self.linPos = rmpVals[x] 
			elif rmpItems[x] == 'differential_wheel_vel_rps': 		
				self.yawRate = -1 * rmpVals[x] #flipped because it works correctly with the robot and the frame
		
		"""
		Segway RMP base tends to drift when stationary, basic 
		filter to ignore low differences and reduce/stop drift
		"""	
		if self.yawRate >= -0.005 and self.yawRate <= 0.005:
			self.yawRate = 0
			
		"""
		Calculate the new pose based on the feedback from the Segway 
		and the time difference from the previous calculation
		"""
		currentTime = self.timeMsgReceived + self.timeMsgReceived * 0.000000001
		
		if self.firstUpdate == 1:
			xPos = self.initXPos
			yPos = self.initYPos
			yaw = self.initYaw
			self.firstUpdate = 0
		elif self.linPos == 0:
			xPos = self.initXPos
			yPos = self.initYPos
			yaw = self.initYaw
		else:
			yaw = self.prevYaw + self.yawRate * (currentTime - self.prevTime)
			xPos = self.prevXPos + (self.linPos - self.prevLinPos) * math.cos(yaw)
			yPos = self.prevYPos + (self.linPos - self.prevLinPos) * math.sin(yaw)

		"""
		Store the current values to be used in the next iteration
		"""
		self.prevLinPos = self.linPos
		self.prevXPos = xPos
		self.prevYPos = yPos
		self.prevYaw = yaw
		self.prevTime = currentTime

		"""
		create quaternion array from rmp and imu data
		"""
		quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
		
		"""
		Make and publish the odometry message
		"""
		odom = Odometry()
		odom.header.stamp = rospy.Time.now()
		odom.header.frame_id = "odom"
		odom.child_frame_id = "base_footprint"
		odom.pose.pose.position.x = xPos
		odom.pose.pose.position.y = yPos
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation.x = quat[0]
		odom.pose.pose.orientation.y = quat[1]
		odom.pose.pose.orientation.z = quat[2]
		odom.pose.pose.orientation.w = quat[3]
		odom.twist.twist.linear.x = self.linVel
		odom.twist.twist.linear.y = 0 #Segway is differential drive therefore always zero
		odom.twist.twist.angular.z = self.yawRate

		self.odomPub.publish(odom)
		 
		"""
		publish the transform from odom to the base footprint
		"""
		self.tfBroadCast.sendTransform((xPos, yPos, 0), quat, rospy.Time.now(), '/base_footprint', '/odom')
			
	"""
	Main loop for this class, used to initialize the node
	"""
	def rmp_data_receive(self):
		rospy.init_node('pose_update')
		rospy.Subscriber("rmp_feedback", rmpFeedback, self.pose_update)
		print "Pose update node started."
		rospy.spin()

if __name__ == "__main__":
	poseUpdate = PoseUpdate()
	poseUpdate.rmp_data_receive()
