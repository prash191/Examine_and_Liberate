#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from operator import index
from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
from survey_and_rescue.msg import*
import rospy
import time
import sys, select, termios, tty
import json

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control
		self.detect_info_msg = SRInfo()

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	
		self.data = {}
		with open("/home/prashant/catkin_ws/src/Examine_and_Liberate/survey_and_rescue/scripts/cell_coords.json", "r") as read_file:
			self.data = json.load(read_file)

		self.setpoint = []
		
	

		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		# self.Kp = [10, 9, 40]
		# self.Ki = [0.5, 0.4, 0]
		# self.Kd = [375,370,340]
		# {"A1": [-7.524, -8.883, 24.068], "A3": [-8.169, -2.404, 26.456], "A2": [-7.895, -5.835, 25.595],

# veryyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy imppppppppppppppppppppppppppp
		# self.Kp = [15, 13, 45]
		# self.Ki = [0.2, 0.2, 0.1]
		# self.Kd = [320,320,240]

		# self.Kp = [10, 10, 45]
		# self.Ki = [0, 0, 0]
		# self.Kd = [340,340,240]

		# self.Kp = [[8, 8, 42], [15, 32, 45]]
		# self.Ki = [[0, 0, 0], [0, 0, 0]]
		# self.Kd = [[350,350,270], [320, 320, 240]]

		self.Kp = [[15, 15, 45], [15, 32, 45]]
		self.Ki = [[0.2, 0.2, 0.1], [0, 0, 0]]
		self.Kd = [[340,340,240], [320, 320, 240]]

		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.max_value = [2000, 2000, 2000]
		self.min_value = [1000, 1000, 1000]
		self.prev_values = [0,0,0]




		
		self.sample_time = 0.050 # in seconds
		self.cur_time = 0.00
		self.prev_time = 0.00
		self.prev_hold_time = 0.00





		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)

		
		#------------------------Add other ROS Publishers here-----------------------------------------------------

		#me
		self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)

		#-------------------------------------------------------------------------------------------------------------
		#me
		self.error = [0,0,0]
		self.prev_error = [0,0,0]
		self.sum_of_error = [0,0,0]
		self.change_in_error = [0,0,0]
		# self.Iterm = 0


		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		
		#*********************************for last task
		rospy.Subscriber('/decision_info',SRInfo,self.decision_callback)

		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		#me
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)



		

		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE




	#*********************************for last task
	def decision_callback(self,msg):
		self.out=[0,0,0]
		self.Iterm=[0,0,0]
		print(msg.location)
		self.setpoint = self.data[msg.location]
		print(self.setpoint)
		# self.setpoint=self.cell_coords.get(msg.location)



	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(0.5)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):
		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(0.5)


	def land(self):
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(0.5)

	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):

		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z

		
	#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This functirinton gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	#me
	def pitch_set_pid(self,pitch):
		self.Kp[1] = pitch.Kp * 0.06 
		self.Ki[1] = pitch.Ki * 0.008
		self.Kd[1] = pitch.Kd * 0.3

	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp * 0.06
		self.Ki[0] = roll.Ki * 0.008
		self.Kd[0] = roll.Kd * 0.3




	#----------------------------------------------------------------------------------------------------------------------

		
	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------
		# self.cur_time = rospy.get_time()
		# if(self.cur_time - self.prev_time >= self.sample_time):

		# Steps:
		# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
		
			if(self.setpoint):

				for i in range(0,3):
					self.error[i] = self.drone_position[i] - self.setpoint[i]

				for i in range(0,3):
					self.sum_of_error[i]+=self.error[i] 

			#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
				for i in range(0,3):
					self.change_in_error[i] = self.error[i] - self.prev_error[i]

				

			#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
				
				id = 0
				# if(abs(self.error[0])>6.7 or abs(self.error[1])>7.2):
				# 	id = 0
				# else:
				# 	id=1

				self.out_roll  = self.Kp[id][0]*self.error[0] + (self.sum_of_error[0]) * self.Ki[id][0] + self.Kd[id][0]*self.change_in_error[0]
				self.out_pitch = self.Kp[id][1]*self.error[1] + (self.sum_of_error[1]) * self.Ki[id][1] + self.Kd[id][1]*self.change_in_error[1]
				self.out_alt   = self.Kp[id][2]*self.error[2] + (self.sum_of_error[2]) * self.Ki[id][2] + self.Kd[id][2]*self.change_in_error[2]
				
			#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
				self.cmd.rcRoll=1500-self.out_roll
				self.cmd.rcPitch=1500+self.out_pitch
				self.cmd.rcThrottle=1500+self.out_alt
			#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sample_time defined above is for this purpose. THIS IS VERY IMPORTANT.

			#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
			#																														self.cmd.rcPitch = self.max_values[1]
				if self.cmd.rcRoll > self.max_value[0]:
					self.cmd.rcRoll = self.max_value[0]

				if self.cmd.rcRoll < self.min_value[0]:
					self.cmd.rcRoll = self.min_value[0]
				
				if self.cmd.rcPitch > self.max_value[1]:
					self.cmd.rcPitch = self.max_value[1]

				if self.cmd.rcPitch < self.min_value[1]:
					self.cmd.rcPitch = self.min_value[1]

				if self.cmd.rcThrottle > self.max_value[2]:
					self.cmd.rcThrottle = self.max_value[2]

				if self.cmd.rcThrottle < self.min_value[2]:
					self.cmd.rcThrottle = self.min_value[2]


			#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
				for i in range(0,3):
					self.prev_error[i] = self.error[i]
			#	8. Add error_sum= 
				

				self.cur_time = rospy.get_time()

				if(self.cur_time - self.prev_time >= self.sample_time):
					
					self.prev_time = self.cur_time
					for i in range(0,3):
						self.sum_of_error[i] = 0
					
				self.command_pub.publish(self.cmd)
				self.roll_error_pub.publish(self.error[0])
				self.pitch_error_pub.publish(self.error[1])
				self.alt_error_pub.publish(self.error[2])

	# def waypoint(self, id):
	# 	if id == len(self.setpoints):
	# 		return
	# 	self.setpoint = self.setpoints[id]
	# 	current_time = rospy.get_time()
	# 	# current_time =  rospy.get_rostime()

	# 	if current_time != 0 and self.prev_hold_time == 0 and self.setpoint_index == 0:
	# 		self.prev_hold_time = current_time
		
	# 	# print("self.setpoint = ", self.val[id])
	# 	# print("current_time = ",current_time)
	# 	# print("self.prev_hold_time = ", self.prev_hold_time)
	# 	# print()

	# 	if current_time - self.prev_hold_time >= 12:
	# 		self.prev_hold_time = current_time
	# 		self.setpoint_index+=1




if __name__ == '__main__':

	settings = termios.tcgetattr(sys.stdin)
	e_drone = Edrone()
	r = rospy.Rate(20)
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()
		key = getKey()
		if key == '\x03':
			break
	e_drone.land()
	e_drone.disarm()
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


	
