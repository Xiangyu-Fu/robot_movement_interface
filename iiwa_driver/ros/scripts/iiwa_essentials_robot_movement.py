#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Copyright 2015 Fraunhofer IPA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# -----------------------------------------------------------------------------
# This node provides the Robot Movement Interface as well as state publishing
# - command list topic is used to control the robot
# - command result topic provides feedback after command completion
# - state publishing (tool, flang, joints, force, torque)
# The driver consists on two threads and a callback:
# 	Thread 1: state publishing: it peridically publishes robot state (100 Hz)
#	Thread 2: robot controlling: polling (100 Hz) to wait for new commands in the pipeline
#             if there are new commands, each will be sequentially executed but only after the termination of the current one
#	Command callback: adds the received commands to the pipeline. It is also possible to erase not executed commands from the pipeline
# -----------------------------------------------------------------------------
import math
import rospy
import _thread as thread
import copy
import socket
import time
from geometry_msgs.msg			   import WrenchStamped
from sensor_msgs.msg               import JointState
from trajectory_msgs.msg           import *
from iiwa_driver.srv 	           import *
from robot_movement_interface.msg  import *
# ------------------------------------------------------------------------------------
# Notice that joint direct moves can be max. 0.087 radians away per axis (Kuka)  
# ------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------
# Main configuration - This default configuration will be overwritten by the config.yaml file
# ------------------------------------------------------------------------------------
conf_controlling_rate 			= 100 				# Hz for controlling 
conf_publishing_rate			= 100				# Hz for publishing robot state
conf_com_node     				= 'iiwa_telnet'		# Communication topic to sent the robot commands
conf_sub_topic_command_list		= 'command_list'	# Topic in which commands are received
conf_pub_topic_command_result 	= 'command_result' 	# Topic which gives command feedback
# ------------------------------------------------------------------------------------
# Published topics - This default configuration will be overwritten by the config.yaml file
# ------------------------------------------------------------------------------------
conf_pub_topic_joints		= 'joint_states'	# Joint configuration publishing
conf_pub_topic_wrench 		= 'tcp_wrench'		# Force and torque publishing
conf_pub_topic_iiwa_frame	= 'tool_frame' 		# Frame publishing as iiwa frame
conf_pub_topic_iiwa_flange	= 'flange_frame'	# Flange frame publishing as iiwa frame
conf_pub_topic_robot_state	= 'robot_state' 	# Current robot state
# ------------------------------------------------------------------------------------
# Additional configuration - This default configuration will be overwritten by the config.yaml file
# ------------------------------------------------------------------------------------
conf_robot_base_name = 'base_link'
conf_joint_names  = ['joint_0','joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
conf_max_speed    = [1.71042267, 1.71042267, 1.74532925, 2.26892803, 2.44346095, 3.14159265, 3.14159265] # rad / s joint_0..joint_1

conf_cartesian_delta = 0.001 # Next position will be send to the robot x m in advance
conf_joints_delta = 0.001 # 0.01 radians

# ---------------------------------------------------------------------------------------
# Shared variables - Thread safe
# ---------------------------------------------------------------------------------------
lock 				= thread.allocate_lock() 	# Lock for shared variables
lockCom 			= thread.allocate_lock() 	# Lock for communaication node topic sharing
current_joints    	= []
current_effort    	= []
current_velocity  	= []
current_tcp_frame   = []
current_tcp_force   = []
current_tcp_torque  = []
command_list 		= [] 						# Contains the command list to be executed
command_active   	= None 						# Current active command
command_last 		= None 						# Last finished command


# -----------------------------------------------------------------------------
# Default parameters, they will be overwritten by the config.yaml file
# -----------------------------------------------------------------------------
TCP_IP = "10.200.2.84"
TCP_PORT = 30000
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# -----------------------------------------------------------------------------
# Connect with the robot. It will try to reconnect in case of error
# -----------------------------------------------------------------------------
def socket_connect():
	while not rospy.is_shutdown():
		try:
			rospy.logwarn('Connecting to ' + TCP_IP + ":" + str(TCP_PORT))
			sock.connect((TCP_IP, TCP_PORT))
			rospy.logwarn('Connected to ' + TCP_IP + ":" + str(TCP_PORT))
			break
		except:	
			time.sleep(1)

def flush(line):
	while not rospy.is_shutdown():
		try:
			sock.send(line.encode())
			# rospy.logwarn("Socket send:"+line.encode())
			# rsl = sock.recv(1)
			# rospy.logwarn("get respond")
			# rospy.logwarn('Received: ' + rsl.decode())
			# return rsl.decode()
			return True
		except:
			socket_connect()

# ---------------------------------------------------------------------------------------
# Move manager. This node does a polling waiting for new commands and executing then when available
# ---------------------------------------------------------------------------------------
def move_manager(rate):

	# Path variables
	global command_list		# Command pipeline
	global command_active	# Current active command
	global command_last		# Last executed command

	# If last command was a Direct Mode command, then we need to keep alive by resending the command (Kuka feature)

	#com = rospy.ServiceProxy(conf_com_node, StringCommand, persistent=True) # IMPORTANT: Service calling must be locked, otherwise message order is not preserved

	# Command result feedback after command execution
	#publisher_command_result = rospy.Publisher(conf_pub_topic_command_result, Result, queue_size = conf_publishing_rate)

	sleeper = rospy.Rate(rate)

	while not rospy.is_shutdown():
		# -----------------------------------------------------------
		# Critical Section
		# -----------------------------------------------------------
		with lock:
			#code = 0
			# There is an active command, check if it is finished
			# if command_active:
			# 	if True:
			# 		# Blending can be default or overwritten by the command
			# 		blending = 0.0 # 1 mm
			# 		if len(command_active.blending) > 0:
			# 			blending = command_active.blending[0]
			# 		if len(command_active.blending) > 1:
			# 			delta = command_active.blending[1]

			# 		# default result code is success
			# 		# result code: 0 - success
			# 		# result code: 100 - error (not implemented)
			# 		# result code: 200 - collision
			# 		result_code = 0

			# 		# check if the target was reached when using force threshold commands or if there was a collision
			# 		if command_active.command_type == "PTPFORCE" or command_active.command_type == "LINFORCE":
			# 			# when target was not reached with the configuratiton
			# 			if not multiDimensionalDistance(current_tcp_frame[:3], command_active.pose[:3], blending + conf_cartesian_delta):							
			# 				result_code = 200

            #         # Keep alive direct commands by coping the command again in the pipeline (DIRECT and SMART mode only)
			# 		if len(command_list) == 0:
			# 			if command_active.command_type == 'DIRECT' or command_active.command_type == 'SMART':
			# 				command_list.insert(0, command_active)
			# 		# Advance 1 in the pipeline
			# 		command_last = command_active
			# 		command_active = None
			# 		# Publish message feedback
			# 		command_result_msg = Result()
			# 		command_result_msg.command_id = command_last.command_id
			# 		command_result_msg.result_code = result_code
			# 		# publisher_command_result.publish(command_result_msg)
			# There is currently no active command, so execute next		
			# else:
			# 	if command_list:
			# 		command_active = command_list.pop(0)
			# 		rospy.logwarn("Executing command: " + command_active.command_type + command_active.pose_type + " " + str(command_active.pose))
			# 		executeCommand(command_active)
		
			if command_list:
				command_active = command_list.pop(0)
				# rospy.logwarn("Executing command: " + command_active.command_type + " "+ command_active.pose_type + " " + str(command_active.pose))
				executeCommand(command_active)
		
		# rospy.logwarn("move loop")
		sleeper.sleep()
		# -----------------------------------------------------------
		# sleeper.sleep()
		# -----------------------------------------------------------

# ---------------------------------------------------------------------------------------
# Executes a new command
# ---------------------------------------------------------------------------------------
def executeCommand(command):

	if command.command_type == 'LINIMPEDENCE' and command.pose_type == 'EULER_INTRINSIC_ZYX':
		assert len(command.pose) == 6 and len(command.velocity) == 1 and len(command.force_threshold) == 3
		with lockCom:
			temp_pose = [command.pose[0] * 1000.0, command.pose[1] * 1000.0, command.pose[2] * 1000.0, command.pose[3], command.pose[4], command.pose[5]]
			temp_force_threshold = [command.force_threshold[0], command.force_threshold[1], command.force_threshold[2]]
			torque_threshold = [0.1, 0.1, 0.1]
			# rospy.logwarn("sending command")

			# response = StringCommandResponse()	# ROS StringCommandResponse service response
			# response.error_code = 0				# No error by default
			# temp_pose = [X, Y, Z, Alpha, Beta, Gamma, Velocity, ForceThresholdX, ForceThresholdY, ForceThresholdZ, TorqueThresholdX, ThresholdY, ThresholdZ]
			# com('linstiff move', ' '.join(str(i) for i in temp_pose) + ' ' + str(command.velocity[0]) + ' ' + '700' + ' ' + ' '.join(str(i) for i in temp_force_threshold) + ' ' + ' '.join(str(i) for i in torque_threshold))
			flush('linstiff move' + " : " + " ".join(str(i) for i in temp_pose) + ' ' + str(command.velocity[0]) + ' ' + '700' + ' ' + ' '.join(str(i) for i in temp_force_threshold) + ' ' + ' '.join(str(i) for i in torque_threshold) + "\n" ) 			
			# rospy.logwarn('linstiff move' + " : ".join(str(i) for i in temp_pose) + ' ' + str(command.velocity[0]) + ' ' + '700' + ' ' + ' '.join(str(i) for i in temp_force_threshold) + ' ' + ' '.join(str(i) for i in torque_threshold) + "\n" )
				
# ---------------------------------------------------------------------------------------
# Subscription to joint_path_command (trajectory_msgs/JointTrajectory)
# Updates the current trajectory pipeline
# ---------------------------------------------------------------------------------------
def commands_callback(msg):

	global command_list

	# -----------------------------------------------------------
	# Critical Section 
	# -----------------------------------------------------------
	with lock:
		if msg.replace_previous_commands:
			command_list = msg.commands
		else:
			command_list.extend(msg.commands)
		# rospy.logwarn("command received")
	# -----------------------------------------------------------

# ---------------------------------------------------------------------------------------
# Overwrite default parameters with the config.yaml configuration
# ---------------------------------------------------------------------------------------
def loadParameters():

	global conf_controlling_rate
	global conf_publishing_rate
	global conf_com_node
	global conf_sub_topic_command_list
	global conf_pub_topic_command_result
	global conf_pub_topic_joints
	global conf_pub_topic_wrench
	global conf_pub_topic_iiwa_frame
	global conf_pub_topic_iiwa_flange
	global conf_pub_topic_robot_state
	global conf_robot_base_name
	global conf_joint_names
	global conf_max_speed
	global conf_cartesian_delta
	global conf_joints_delta
	
	conf_controlling_rate =  rospy.get_param('~controlling_rate', conf_controlling_rate)
	conf_publishing_rate = rospy.get_param('~publishing_rate', conf_publishing_rate)
	
	conf_com_node = rospy.get_param('~commander_service_name', conf_com_node)
	conf_sub_topic_command_list	= rospy.get_param('~command_list', conf_sub_topic_command_list)
	conf_pub_topic_command_result = rospy.get_param('~command_result', conf_pub_topic_command_result)

	conf_pub_topic_joints = rospy.get_param('~topic_joints', conf_pub_topic_joints)
	conf_pub_topic_wrench = rospy.get_param('~topic_wrench', conf_pub_topic_wrench)
	conf_pub_topic_iiwa_frame = rospy.get_param('~topic_iiwa_frame', conf_pub_topic_iiwa_frame)
	conf_pub_topic_iiwa_flange = rospy.get_param('~topic_iiwa_flange', conf_pub_topic_iiwa_flange)
	conf_pub_topic_robot_state = rospy.get_param('~topic_robot_state', conf_pub_topic_robot_state)
	
	conf_robot_base_name = rospy.get_param('~robot_base_name', conf_robot_base_name)
	conf_joint_names = rospy.get_param('~joint_names', conf_joint_names)
	conf_max_speed = rospy.get_param('~max_speed', conf_max_speed)
	
	conf_cartesian_delta = rospy.get_param('~cartesian_delta', conf_cartesian_delta)
	conf_joints_delta = rospy.get_param('~joints_delta', conf_joints_delta)

# ---------------------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------------------
if __name__ == '__main__':
	try:

		rospy.init_node('iiwa_robot_movement', log_level=rospy.DEBUG)
		
		loadParameters()

		# Load variables from config.yaml
		TCP_IP = rospy.get_param('~robot_ip', TCP_IP)
		TCP_PORT = rospy.get_param('~robot_port', TCP_PORT)

		rospy.loginfo("Robot IP: " + TCP_IP)
		
		# SUNRISE communication node
		socket_connect()
		#rospy.wait_for_service(conf_com_node)

		#thread.start_new_thread(publishers,(conf_publishing_rate,))
		thread.start_new_thread(move_manager,(conf_controlling_rate,))

		rospy.Subscriber(conf_sub_topic_command_list, CommandList, commands_callback)
		rospy.spin()

	except rospy.ROSInterruptException:
		pass

