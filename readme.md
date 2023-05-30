# Automatic Control
 The provided code simulates the behavior of a reference and tracker robot system and applies different control methods to the tracker based on the specified control mode. The unicycle model is used to represent the motion of the robots, and the control methods implemented are linear control, Lyapunov control, and PID control. The code reads input data from the `input.yaml` file to initialize the system and control parameters. The simulation results are visualized using `matplotlib.pyplot`.

## Input file
Contains the input data for the simulation:
* Initial state of the reference robot (`ref_state`).
* Initial state of the tracker robot (`tr_state`).
* Constant controls of the reference robot (`v_ref` and `w_ref`).
* Control method to be used (`control`). It can be one of the following:
    * 0: Linear control.
    * 1: Lyapunov control.
    * 2: PID control.
    * A combination of 2 control methods (e.g., [2, 0] for PID followed by linear control).

## Main
### Imports
* `numpy` (as `np`): A library for numerical operations in Python.
* `matplotlib.pyplot` (as `plt`): A plotting library for creating visualizations in Python.
* `yaml`: A library for working with YAML files in Python.
* `Unicycle`: A custom class defined in Unicycle.py.
* `Control`: A custom class defined in Control.py.

### Code
* Reads input data from a YAML file named `input.yaml` and stores it in the input_data variable.
* Defines the reference velocity (`V_ref`) and turn rate (`W_ref`) based on the input data.
* Creates instances of the Unicycle class for the reference and tracker systems, using the reference and tracker states from the input data.
* Initializes the controller by creating an instance of the Control class, using the specified control mode and reference and tracker states.
* Stores the initial error from the controller in the variable `error_t0`.
* Initializes containers to store the states of the reference and tracker systems.
* Simulates the systems for 5000 iterations (dt=.01 sec):
    * Updates the reference system's state based on the reference velocity and turn rate.
    * Computes the control signal using the controller and the states of the reference and tracker systems.
    * Updates the tracker system's state using the computed control signal.
    * Stores the states of the reference and tracker systems in their respective containers.
* Prints the initial and final errors.
* Plots the trajectories of the reference and tracker systems using matplotlib.pyplot.
* Plots the error evolution in the X, Y, and Theta dimensions.


## Unicycle
### Imports
* `numpy` (as `np`): A library for numerical operations in Python.

### Class: Unicycle
* Methods:
    * `__init__(self, state0)`: Initializes an instance of the unicycle class with the initial state.
    * `update(self, v, w)`: Updates the unicycle's state based on the velocity (`v`) and turn rate (`w`).

## Control
### Imports
* `numpy` (as `np`): A library for numerical operations in Python.

### Class: Control
* Methods:
    * `__init__(self, mode, state_ref, state_tr)`: Initializes an instance of the control class with the control mode, reference state, and tracker state.
    * `initialize_pid(self, error_t0)`: Initializes the PID control variables with the initial error.
    * `error(self, state_ref, state_tr)`: Calculates the error between the reference and tracker states.
    * `lyapunov_control(self, error, refV, refW)`: Computes the control signal using Lyapunov control.
    * `linear_control(self, error)`: Computes the control signal using linear control.
    * `pid_control(self, error)`: Computes the control signal using PID control.
    * `control_switch(self, error, control_ref, mode)`: Returns the control signal based on the control mode.
    * `control(self, state_ref, control_ref, state_tr)`: Controls the tracker system based on the reference and tracker states.
