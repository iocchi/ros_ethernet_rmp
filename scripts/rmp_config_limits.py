"""
File used to determine the commands sent to the robot are valid commands. 
The file organizes the parameters defined in python_ethernet_rmp.system_defines.py
into dictionaries

Author:  Chris Dunkers, Worcester Polytechnic Institute
Author:  Russell Toris, Worcester Polytechnic Institute
Version: June 11, 2014
"""

from python_ethernet_rmp.system_defines import *
from python_ethernet_rmp.utils import *
import rospy
import sys

"""
Function to check the range of the parameter value. 
If out of range the parameter is set as the boundary
"""
def isRangeSet(arg1, arg2):
	if arg2 > config_params_max[arg1]:
		rospy.logwarn("%s over limit. Set to %s", config_params_names[arg1], config_params_max[arg1])
		return [convert_float_to_u32(config_params_max[arg1]),True]
	elif arg2 < config_params_min[arg1]:
		rospy.logwarn("%s under limit. Set to %s", config_params_names[arg1], config_params_min[arg1])
		return [convert_float_to_u32(config_params_min[arg1]),True]
	else:
		return [convert_float_to_u32(arg2),True]
"""
Function to check the range of the parameter value. 
If out of range the parameter is not sent to the robot
"""
def isRangeNoSet(arg1, arg2):
	if arg2 > config_params_max[arg1]:
		rospy.logwarn("%s over limit. Command not sent.", config_params_names[arg1], config_params_max[arg1])
		return [arg2,False]
	elif arg2 < config_params_min[arg1]:
		rospy.logwarn("%s under limit. Command not sent.", config_params_names[arg1], config_params_min[arg1])
		return [arg2,False]
	else:
		return [arg2,True]

"""
Function to check the range of the parameter value. 
If out of range the parameter is not sent to the robot
"""
def isRange(arg1, arg2):
	if arg2 > config_params_max[arg1]:
		rospy.logwarn("%s over limit. Command not sent.", config_params_names[arg1], config_params_max[arg1])
		return [int(arg2),False]
	elif arg2 < config_params_min[arg1]:
		rospy.logwarn("%s under limit. Command not sent.", config_params_names[arg1], config_params_min[arg1])
		return [int(arg2),False]
	else:
		return [int(arg2),True]
	
"""
Checks to make sure the IP info is properly configured
"""
def isQuad(arg1, arg2):
	try:
		return [dottedQuadToNum(arg2),True]
	except:
		rospy.logwarn("%s is invalid. Command not sent.",config_params_names[arg1])
		return [arg2,False]

"""
Returns True as the command is always valid
"""
def isTrue(arg1, arg2):
	return [arg2, True]

"""
Prints a rosparam error as the command does not exist
"""
def isNotParam(arg1, arg2):
	rospy.logwarn("arg1 is invalid. Command not sent.")
	return [arg2,False]

"""
Dictionary to define the Maximum values
"""
config_params_max = dict({ 
					RMP_CMD_NONE:0,
					RMP_CMD_SET_MAXIMUM_VELOCITY:MAX_VELOCITY_MPS,
					RMP_CMD_SET_MAXIMUM_ACCELERATION:MAX_ACCELERATION_MPS2,
					RMP_CMD_SET_MAXIMUM_DECELERATION:MAX_DECELERATION_MPS2,
					RMP_CMD_SET_MAXIMUM_DTZ_DECEL_RATE:MAX_DTZ_DECEL_RATE_MPS2,
					RMP_CMD_SET_COASTDOWN_ACCEL:MAX_COASTDOWN_ACCEL_MPS2,
					RMP_CMD_SET_MAXIMUM_TURN_RATE:MAX_YAW_RATE_RPS,
					RMP_CMD_SET_MAXIMUM_TURN_ACCEL:MAX_YAW_ACCEL_RPS2,
					RMP_CMD_SET_TIRE_DIAMETER:MAX_TIRE_DIAMETER_M,
					RMP_CMD_SET_WHEEL_BASE_LENGTH:MAX_WHEEL_BASE_LENGTH_M,
					RMP_CMD_SET_WHEEL_TRACK_WIDTH:MAX_WHEEL_TRACK_WIDTH_M,
					RMP_CMD_SET_TRANSMISSION_RATIO:MAX_TRANSMISSION_RATIO,
					RMP_CMD_SET_INPUT_CONFIG_BITMAP:4294967295,
					RMP_CMD_SET_ETH_IP_ADDRESS:None,
					RMP_CMD_SET_ETH_PORT_NUMBER:65535,
					RMP_CMD_SET_ETH_SUBNET_MASK:None,
					RMP_CMD_SET_ETH_GATEWAY:None,
					RMP_CMD_SET_USER_FB_1_BITMAP:4294967295,
					RMP_CMD_SET_USER_FB_2_BITMAP:4294967295,
					RMP_CMD_SET_USER_FB_3_BITMAP:4294967295,
					RMP_CMD_SET_USER_FB_4_BITMAP:0,
					RMP_CMD_SET_AUDIO_COMMAND:16,
					RMP_CMD_SET_OPERATIONAL_MODE:6,
					RMP_CMD_SEND_SP_FAULTLOG:1,
					RMP_CMD_RESET_INTEGRATORS:31,
					RMP_CMD_RESET_PARAMS_TO_DEFAULT:0,
					RMP_CMD_FORCE_CONFIG_FEEDBACK_BITMAPS:1})
"""
Dictionary to define the Minimum values
"""
config_params_min = dict({ 
					RMP_CMD_NONE:0,
					RMP_CMD_SET_MAXIMUM_VELOCITY:MIN_VELOCITY_MPS,
					RMP_CMD_SET_MAXIMUM_ACCELERATION:MIN_ACCELERATION_MPS2,
					RMP_CMD_SET_MAXIMUM_DECELERATION:MIN_DECELERATION_MPS2,
					RMP_CMD_SET_MAXIMUM_DTZ_DECEL_RATE:MIN_DTZ_DECEL_RATE_MPS2,
					RMP_CMD_SET_COASTDOWN_ACCEL:MIN_COASTDOWN_ACCEL_MPS2,
					RMP_CMD_SET_MAXIMUM_TURN_RATE:MIN_YAW_RATE_RPS,
					RMP_CMD_SET_MAXIMUM_TURN_ACCEL:MIN_YAW_ACCEL_RPS2,
					RMP_CMD_SET_TIRE_DIAMETER:MIN_TIRE_DIAMETER_M,
					RMP_CMD_SET_WHEEL_BASE_LENGTH:MIN_WHEEL_BASE_LENGTH_M,
					RMP_CMD_SET_WHEEL_TRACK_WIDTH:MIN_WHEEL_TRACK_WIDTH_M,
					RMP_CMD_SET_TRANSMISSION_RATIO:MIN_TRANSMISSION_RATIO,
					RMP_CMD_SET_INPUT_CONFIG_BITMAP:0,
					RMP_CMD_SET_ETH_IP_ADDRESS:None,
					RMP_CMD_SET_ETH_PORT_NUMBER:0,
					RMP_CMD_SET_ETH_SUBNET_MASK:None,
					RMP_CMD_SET_ETH_GATEWAY:None,
					RMP_CMD_SET_USER_FB_1_BITMAP:0,
					RMP_CMD_SET_USER_FB_2_BITMAP:0,
					RMP_CMD_SET_USER_FB_3_BITMAP:0,
					RMP_CMD_SET_USER_FB_4_BITMAP:0,
					RMP_CMD_SET_AUDIO_COMMAND:0,
					RMP_CMD_SET_OPERATIONAL_MODE:1,
					RMP_CMD_SEND_SP_FAULTLOG:0,
					RMP_CMD_RESET_INTEGRATORS:0,
					RMP_CMD_RESET_PARAMS_TO_DEFAULT:0,
					RMP_CMD_FORCE_CONFIG_FEEDBACK_BITMAPS:0})

"""
Dictionary to define the parameter names for error messages
"""					
config_params_names = dict({ 
					RMP_CMD_NONE:"RMP_CMD_NONE",
					RMP_CMD_SET_MAXIMUM_VELOCITY:"RMP_CMD_SET_MAXIMUM_VELOCITY",
					RMP_CMD_SET_MAXIMUM_ACCELERATION:"RMP_CMD_SET_MAXIMUM_ACCELERATION",
					RMP_CMD_SET_MAXIMUM_DECELERATION:"RMP_CMD_SET_MAXIMUM_DECELERATION",
					RMP_CMD_SET_MAXIMUM_DTZ_DECEL_RATE:"RMP_CMD_SET_MAXIMUM_DTZ_DECEL_RATE",
					RMP_CMD_SET_COASTDOWN_ACCEL:"RMP_CMD_SET_COASTDOWN_ACCEL",
					RMP_CMD_SET_MAXIMUM_TURN_RATE:"RMP_CMD_SET_MAXIMUM_TURN_RATE",
					RMP_CMD_SET_MAXIMUM_TURN_ACCEL:"RMP_CMD_SET_MAXIMUM_TURN_ACCEL",
					RMP_CMD_SET_TIRE_DIAMETER:"RMP_CMD_SET_TIRE_DIAMETER",
					RMP_CMD_SET_WHEEL_BASE_LENGTH:"RMP_CMD_SET_WHEEL_BASE_LENGTH",
					RMP_CMD_SET_WHEEL_TRACK_WIDTH:"RMP_CMD_SET_WHEEL_TRACK_WIDTH",
					RMP_CMD_SET_TRANSMISSION_RATIO:"RMP_CMD_SET_TRANSMISSION_RATIO",
					RMP_CMD_SET_INPUT_CONFIG_BITMAP:"RMP_CMD_SET_INPUT_CONFIG_BITMAP",
					RMP_CMD_SET_ETH_IP_ADDRESS:"RMP_CMD_SET_ETH_IP_ADDRESS",
					RMP_CMD_SET_ETH_PORT_NUMBER:"RMP_CMD_SET_ETH_PORT_NUMBER",
					RMP_CMD_SET_ETH_SUBNET_MASK:"RMP_CMD_SET_ETH_SUBNET_MASK",
					RMP_CMD_SET_ETH_GATEWAY:"RMP_CMD_SET_ETH_GATEWAY",
					RMP_CMD_SET_USER_FB_1_BITMAP:"RMP_CMD_SET_USER_FB_1_BITMAP",
					RMP_CMD_SET_USER_FB_2_BITMAP:"RMP_CMD_SET_USER_FB_2_BITMAP",
					RMP_CMD_SET_USER_FB_3_BITMAP:"RMP_CMD_SET_USER_FB_3_BITMAP",
					RMP_CMD_SET_USER_FB_4_BITMAP:"RMP_CMD_SET_USER_FB_4_BITMAP",
					RMP_CMD_SET_AUDIO_COMMAND:"RMP_CMD_SET_AUDIO_COMMAND",
					RMP_CMD_SET_OPERATIONAL_MODE:"RMP_CMD_SET_OPERATIONAL_MODE",
					RMP_CMD_SEND_SP_FAULTLOG:"RMP_CMD_SEND_SP_FAULTLOG",
					RMP_CMD_RESET_INTEGRATORS:"RMP_CMD_RESET_INTEGRATORS",
					RMP_CMD_RESET_PARAMS_TO_DEFAULT:"RMP_CMD_RESET_PARAMS_TO_DEFAULT",
					RMP_CMD_FORCE_CONFIG_FEEDBACK_BITMAPS:"RMP_CMD_FORCE_CONFIG_FEEDBACK_BITMAPS"})

"""
Dictionary to define the checking function, defined above, to be called 
for each valid parameter
"""
config_params_function = dict({ 
					RMP_CMD_NONE:isTrue,
					RMP_CMD_SET_MAXIMUM_VELOCITY:isRangeSet,
					RMP_CMD_SET_MAXIMUM_ACCELERATION:isRangeSet,
					RMP_CMD_SET_MAXIMUM_DECELERATION:isRangeSet,
					RMP_CMD_SET_MAXIMUM_DTZ_DECEL_RATE:isRangeSet,
					RMP_CMD_SET_COASTDOWN_ACCEL:isRangeSet,
					RMP_CMD_SET_MAXIMUM_TURN_RATE:isRangeSet,
					RMP_CMD_SET_MAXIMUM_TURN_ACCEL:isRangeSet,
					RMP_CMD_SET_TIRE_DIAMETER:isRangeSet,
					RMP_CMD_SET_WHEEL_BASE_LENGTH:isRangeSet,
					RMP_CMD_SET_WHEEL_TRACK_WIDTH:isRangeSet,
					RMP_CMD_SET_TRANSMISSION_RATIO:isRangeSet,
					RMP_CMD_SET_INPUT_CONFIG_BITMAP:isRange,
					RMP_CMD_SET_ETH_IP_ADDRESS:isQuad,
					RMP_CMD_SET_ETH_PORT_NUMBER:isRange,
					RMP_CMD_SET_ETH_SUBNET_MASK:isQuad,
					RMP_CMD_SET_ETH_GATEWAY:isQuad,
					RMP_CMD_SET_USER_FB_1_BITMAP:isRange,
					RMP_CMD_SET_USER_FB_2_BITMAP:isRange,
					RMP_CMD_SET_USER_FB_3_BITMAP:isRange,
					RMP_CMD_SET_USER_FB_4_BITMAP:isRange,
					RMP_CMD_SET_AUDIO_COMMAND:isRangeNoSet,
					RMP_CMD_SET_OPERATIONAL_MODE:isRangeNoSet,
					RMP_CMD_SEND_SP_FAULTLOG:isRangeNoSet,
					RMP_CMD_RESET_INTEGRATORS:isRangeNoSet,
					RMP_CMD_RESET_PARAMS_TO_DEFAULT:isTrue,
					RMP_CMD_FORCE_CONFIG_FEEDBACK_BITMAPS:isRangeNoSet})
					


