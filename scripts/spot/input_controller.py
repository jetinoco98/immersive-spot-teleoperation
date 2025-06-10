import numpy as np
from casadi import *
import do_mpc
from matplotlib import rcParams
import matplotlib.pyplot as plt

INPUT_TRESHOLD = 0.5
VELOCITY_BASE_SPEED = 0.5 # m/s
VELOCITY_BASE_ANGULAR = 0.5 # m/s
#MAX_OUTPUT = 0.8 # rad/s

class Controller:
    def __init__(self):
        self.setpoints = np.array([[0],[0]])
        self.model = self.get_model()
        self.lqr = self.get_lqr(self.model)

    def get_model(self):
        model_type = 'discrete' 
        model = do_mpc.model.LinearModel(model_type)
        _x = model.set_variable(var_type='_x', var_name='x', shape=(2,1))
        _u = model.set_variable(var_type='_u', var_name='u', shape=(2,1))
        x_next =  _x + _u
        model.set_rhs('x', x_next)
        model.setup()
        return model

    def get_lqr(self, model, silence_solver = False):
        lqr = do_mpc.controller.LQR(model)
        lqr.set_param(t_step = 0.05)
        lqr.set_param(n_horizon = None) # infinite horizon
        Q = 10*np.identity(2)
        R = np.identity(2)
        Rdelu = np.identity(2)
        lqr.set_objective(Q=Q, R=R)
        lqr.set_rterm(delR = Rdelu)
        lqr.setup()
        return lqr
    
    def compute_setpoints(self, setpoints):
        self.setpoints = np.array([[setpoints[0]], [setpoints[1]]])

    def get_simulator(self, model):
        simulator = do_mpc.simulator.Simulator(model)
        params_simulator = {
            't_step': 0.05
        }
        simulator.set_param(**params_simulator)
        simulator.setup()
        return simulator
    
    def compute_control(self, value, base_speed):
        if abs(value) > INPUT_TRESHOLD:
            return base_speed if value > 0 else -base_speed
        return 0
    
    def get_touch_controls(self, setpoints):
        touch_controls = [
            self.compute_control(setpoints[0], VELOCITY_BASE_SPEED),
            self.compute_control(setpoints[1], -VELOCITY_BASE_SPEED),
            self.compute_control(setpoints[2], -VELOCITY_BASE_ANGULAR),
        ]
        return touch_controls
    
    def get_hmd_controls(self, hmd_inputs, measures):
        self.compute_setpoints(hmd_inputs)
        self.lqr.set_setpoint(xss=self.setpoints,uss=np.array([[0],[0]]))
        x = np.array([[measures[0]],[measures[1]]])
        u = self.lqr.make_step(x)
        # Create the control list
        hmd_controls = []
        hmd_controls.append(round(u[0][0], 3))
        hmd_controls.append(round(u[1][0], 3))
        hmd_controls.append(0)
        return hmd_controls


if __name__ == '__main__':
    print(" LQR sample")
    controller = Controller()
    controller.simulator = controller.get_simulator(controller.model)
    x0 = np.array([[0], [0]])
    controller.simulator.x0 = x0
    #controller.lqr.set_initial_guess() # Use initial state to set the initial guess.
    for k in range(50):
        controller.lqr.set_setpoint(xss=np.array([[sin(k)], [cos(k)]]))
        u0 = controller.lqr.make_step(x0)
        y_next = controller.simulator.make_step(u0)
        x0 = y_next
    # Display the results
    rcParams['axes.grid'] = True
    rcParams['font.size'] = 18
    fig, ax, graphics = do_mpc.graphics.default_plot(controller.lqr.data, figsize=(16,9))
    graphics.plot_results()
    graphics.reset_axes()
    plt.show()
