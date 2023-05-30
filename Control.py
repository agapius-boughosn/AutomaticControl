import numpy as np

class Control:
	def __init__(self, mode, state_ref, state_tr):
		# Control mode: 0: linear, 1: lyapunov, 2: PID
		# Combinations of 2 are possible e.g. [1,0] 
		self.mode = mode
		# PID variables
		self.prev_error = 0
		self.integral_term = 0
		# Time step
		self.dt = 0.01
		# Linear control gain
		self.k_linear = [100,50,50]
		# Lyapunov control gain
		self.k_lyapunov = [100, 50]
		# PID control gains
		self.k_pid = [[10, .5, 2], [0.1,  0.1, 0.1], [1, .5, .5]]
		# Initialize PID variables
		self.initialize_pid(self.error(state_ref, state_tr))

	def initialize_pid(self, error_t0):
		# Initialize previous error term and integral term
		self.prev_error = error_t0
		self.integral_term = np.array([0.,0.,0.]).reshape(error_t0.shape)

	# Error calculation
	def error(self, state_ref, state_tr): 
		rotation = np.matrix([[np.cos(state_ref[2]), np.sin(state_ref[2]), 0],
							  [-np.sin(state_ref[2]), np.cos(state_ref[2]), 0],
						 	  [0, 0, 1]])
		diff = np.matrix([[state_tr[0]-state_ref[0]], [state_tr[1]-state_ref[1]], [state_tr[2]-state_ref[2]]])
		return np.matmul(rotation, diff)
	
	# Lyapunov based control
	def lyapunov_control(self, error, refV, refW):
		return [-self.k_lyapunov[0]*(error[0,0]*np.cos(error[2,0])+error[1,0]*np.sin(error[2,0])), \
	  			 refW + (error[0,0]*refV)/(50*error[2,0]) - self.k_lyapunov[1]*error[2,0]]

	# Linearized system control
	def linear_control(self, error):	
		return [-self.k_linear[0]*error[0,0], -self.k_linear[1]*error[1,0]-self.k_linear[1]*error[2,0]]

	# PID control
	def pid_control(self, error):
		self.integral_term += error*self.dt

		v_control = self.k_pid[0][0]*error[0,0] + self.k_pid[1][0]*(error[0,0]-self.prev_error[0,0])/self.dt + self.k_pid[2][0]*self.integral_term[0,0]
		w1_control = self.k_pid[0][1]*error[1,0] + self.k_pid[1][1]*(error[1,0]-self.prev_error[1,0])/self.dt + self.k_pid[2][1]*self.integral_term[1,0]
		w2_control = self.k_pid[0][2]*error[2,0] + self.k_pid[1][2]*(error[2,0]-self.prev_error[2,0])/self.dt + self.k_pid[2][2]*self.integral_term[2,0]

		return [-v_control, -w1_control-w2_control]

	# Return control based on mode
	def control_switch(self, error, control_ref, mode):
		if mode == 0:
			return self.linear_control(error)
		elif mode == 1:
			return self.lyapunov_control(error, control_ref[0], control_ref[1])
		else:
			return self.pid_control(error)

	# Control tracker
	def control(self, state_ref, control_ref, state_tr):
		error = self.error(state_ref, state_tr)

		# Check if combinations of controllers are demanded (e.g. PID then linear)
		if isinstance (self.mode, list):
			# Start with first control method, switch to second when errors are smaller
			if(abs(error[0,0])>1 or abs(error[1,0])>1 or abs(error[2,0])>np.pi/4):	
				control = self.control_switch(error, control_ref, mode = self.mode[0])
			else:
				control = self.control_switch(error, control_ref, mode = self.mode[1])
		else:
			control = self.control_switch(error, control_ref, mode = self.mode)
		
		# Previous error updated here for usability by main even if PID not initiated
		self.prev_error = error
		
		return control