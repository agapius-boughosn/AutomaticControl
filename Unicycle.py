import numpy as np

class Unicycle:
	# Set initial state and controls
    def __init__(self, state0):
        self.state = np.array(state0)
        self.control = np.array([0,0])
        # Time step
        self.dt = .01
	
	# Discretized update equation
    def update(self, v, w):
        self.state += np.array([v*np.cos(self.state[2]), v*np.sin(self.state[2]), w])*self.dt
        self.state[2] = self.state[2]%(2*np.pi)
        self.control = np.array([v,w])
        


