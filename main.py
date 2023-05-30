import numpy as np
import matplotlib.pyplot as plt
import yaml 

from Unicycle import Unicycle
from Control import Control

with open('input.yaml', 'r') as file:
    input_data = yaml.safe_load(file)

# Define reference velocity and turn rate
V_ref, W_ref = input_data['v_ref'], input_data['w_ref']
# Define reference unicyle
ref = Unicycle([float(_) for _ in input_data['ref_state']])
# Define tracker unicycle
tr  = Unicycle([float(_) for _ in input_data['tr_state']])
# Initialize controller
controller = Control(mode=input_data['control'], state_ref = ref.state, state_tr = tr.state)

# Initial error
error_t0 = controller.prev_error

# Initialize containers
stateArr_reference, stateArr_tracker = np.copy(ref.state).reshape(1,3), np.copy(tr.state).reshape(1,3)

# Simulate
for i in range(5000):
	# Move reference with constant velocity and angular rate
	ref.update(V_ref, W_ref)
	# Compute control
	control = controller.control(ref.state, ref.control, tr.state)
	# Set and apply control of tracker 
	tr.update(control[0], control[1])

	# Fill containers with values
	stateArr_reference = np.vstack((stateArr_reference, ref.state))
	stateArr_tracker = np.vstack((stateArr_tracker, tr.state))

# Check initial and final errors
print('initial error (x,y,theta): ', error_t0.flatten())
print('final error (x,y,theta): ', controller.prev_error.flatten())

# Plot effected trajectories by tracker and reference
plt.plot(stateArr_reference[:,0], stateArr_reference[:,1], label='Reference')
plt.plot(stateArr_tracker[:,0], stateArr_tracker[:,1], label='Tracker')
plt.legend()
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.show()

# Plot error evolution
plt.subplot(3,1,1)
plt.ylabel('X error (m)')
plt.plot(stateArr_reference[:,0]-stateArr_tracker[:,0])
plt.subplot(3,1,2)
plt.ylabel('Y error (m)')
plt.plot(stateArr_reference[:,1]-stateArr_tracker[:,1])
plt.subplot(3,1,3)
plt.ylabel('Theta error (rad)')
plt.plot(stateArr_reference[:,2]-stateArr_tracker[:,2])
plt.show()
