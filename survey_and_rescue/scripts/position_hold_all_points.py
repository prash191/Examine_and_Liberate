# import json

# val = ["A1","A2","A3","A4","A5","A6",
# "B1","B2","B3","B4","B5","B6",
# "C1","C2","C3","C4","C5","C6",
# "D1","D2","D3","D4","D5","D6",
# "E1","E2","E3","E4","E5","E6",
# "F1","F2","F3","F4","F5","F6",
# ]
# setpoint=[]
# with open("cell_coords.json", "r") as read_file:
#     data = json.load(read_file)
# for i in range(0,36):
#     setpoint.append(data[val[i]])
# print(setpoint)


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

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import json


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	
        
        # self.val = ["A1","A2","A3","A4","A5","A6","B1","B2","B3","B4","B5","B6","C1","C2","C3","C4","C5","C6","D1","D2","D3","D4","D5","D6","E1","E2","E3","E4","E5","E6","F1","F2","F3","F4","F5","F6"]

		# [x_setpoint, y_setpoint, z_setpoint]

		self.setpoints = []
        # with open("cell_coords.json", "r") as read_file:
        # self.data = json.load(read_file)
        # for i in range(0,36):
        #     self.setpoint.append(self.data[self.val[i]])

		self.setpoint_index = 0
		self.setpoint = [0,0,0] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


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
		self.Kp = [30,30,50]	
		self.Ki = [10,10,5]	
		self.Kd = [350,350,250]	


		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.max_values = [2000,2000,2000]
		self.min_values = [1000,1000,1000]

		self.prev_values = [0,0,0]




		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]
		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.070 # in seconds
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
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		#me
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)





		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


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
		self.cmd.rcThrottle = 1100
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
		self.cur_time = rospy.get_time()
		if(self.cur_time - self.prev_time >= self.sample_time):

		# Steps:
		# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
		
			for i in range(0,3):
				self.sum_of_error[i] = 0

			for i in range(0,3):
				self.error[i] = self.drone_position[i] - self.setpoint[i]

		#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
			for i in range(0,3):
				self.change_in_error[i] = self.error[i] - self.prev_error[i]

		 	

		#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
			self.out_roll  = self.Kp[0]*self.error[0] + (self.sum_of_error[0] + self.error[0]) * self.Ki[0] + self.Kd[0]*self.change_in_error[0]
			self.out_pitch = self.Kp[1]*self.error[1] + (self.sum_of_error[1] + self.error[1]) * self.Ki[1] + self.Kd[1]*self.change_in_error[1]
			self.out_alt   = self.Kp[2]*self.error[2] + (self.sum_of_error[2] + self.error[2]) * self.Ki[2] + self.Kd[2]*self.change_in_error[2]
			
		#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
			self.cmd.rcRoll=1500-self.out_roll
			self.cmd.rcPitch=1500+self.out_pitch
			self.cmd.rcThrottle=1500+self.out_alt
		#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sample_time defined above is for this purpose. THIS IS VERY IMPORTANT.

		#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
		#																														self.cmd.rcPitch = self.max_values[1]
			if self.cmd.rcRoll > self.max_values[0]:
				self.cmd.rcRoll = self.max_values[0]

			if self.cmd.rcRoll < self.min_values[0]:
				self.cmd.rcRoll = self.min_values[0]
			
			if self.cmd.rcPitch > self.max_values[1]:
				self.cmd.rcPitch = self.max_values[1]

			if self.cmd.rcPitch < self.min_values[1]:
				self.cmd.rcPitch = self.min_values[1]

			if self.cmd.rcThrottle > self.max_values[2]:
				self.cmd.rcThrottle = self.max_values[2]

			if self.cmd.rcThrottle < self.min_values[2]:
				self.cmd.rcThrottle = self.min_values[2]


		#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
			
			for i in range(0,3):
				self.prev_error[i] = self.error[i]
		#	8. Add error_sum= 
			for i in range(0,3):
				self.sum_of_error[i]+=self.error[i] 


			self.prev_time = self.cur_time
			
			self.command_pub.publish(self.cmd)
			self.roll_error_pub.publish(self.error[0])
			self.pitch_error_pub.publish(self.error[1])
			self.alt_error_pub.publish(self.error[2])

	def waypoint(self, id):
		if id == len(self.setpoints):
			return
		self.setpoint = self.setpoints[id]
		current_time = rospy.get_time()
		if current_time != 0 and self.prev_hold_time == 0 and self.setpoint_index == 0:
			self.prev_hold_time = current_time
		print(current_time)
		if current_time - self.prev_hold_time >= 3:
			self.prev_hold_time = current_time
			self.setpoint_index+=1




if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(30) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		if e_drone.setpoint_index == len(e_drone.setpoints):
			e_drone.land()
			break
		e_drone.waypoint(e_drone.setpoint_index)
		e_drone.pid()
		r.sleep()


