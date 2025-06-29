import numpy as np
from casadi import *
import do_mpc
from matplotlib import rcParams
import matplotlib.pyplot as plt


INPUT_THRESHOLD = 0.2  # Threshold for joystick inputs to be considered active
VELOCITY_BASE_SPEED = 0.5  # m/s
VELOCITY_BASE_ANGULAR = 0.5  # rad/s


class HMDController:
    def __init__(self):
        self.setpoints = np.array([[0], [0]])
        self.model = self._get_model()
        self.lqr = self._get_lqr(self.model)

    def _get_model(self):
        model = do_mpc.model.LinearModel('discrete')
        _x = model.set_variable(var_type='_x', var_name='x', shape=(2, 1))
        _u = model.set_variable(var_type='_u', var_name='u', shape=(2, 1))
        x_next = _x + _u
        model.set_rhs('x', x_next)
        model.setup()
        return model

    def _get_lqr(self, model):
        lqr = do_mpc.controller.LQR(model)
        lqr.set_param(t_step=0.05)
        lqr.set_param(n_horizon=None)  # infinite horizon
        Q = 10 * np.identity(2)
        R = np.identity(2)
        Rdelu = np.identity(2)
        lqr.set_objective(Q=Q, R=R)
        lqr.set_rterm(delR=Rdelu)
        lqr.setup()
        return lqr
    
    def get_simulator(self, model):
        simulator = do_mpc.simulator.Simulator(model)
        params_simulator = {
            't_step': 0.05
        }
        simulator.set_param(**params_simulator)
        simulator.setup()
        return simulator

    def compute_controls(self, hmd_inputs, measures):
        self.setpoints = np.array([[hmd_inputs[0]], [hmd_inputs[1]]])
        self.lqr.set_setpoint(xss=self.setpoints, uss=np.array([[0], [0]]))
        x = np.array([[measures[0]], [measures[1]]])
        u = self.lqr.make_step(x)
        return [round(u[0][0], 3), round(u[1][0], 3), 0]


class TouchController:
    def compute_controls(self, inputs):
        return [
            self._compute_control(inputs[0], VELOCITY_BASE_SPEED),
            self._compute_control(inputs[1], -VELOCITY_BASE_SPEED),
            self._compute_control(inputs[2], -VELOCITY_BASE_ANGULAR)
        ]

    def _compute_control(self, value, base_speed):
        if abs(value) > INPUT_THRESHOLD:
            return value * base_speed
        return 0


class Controller:
    def __init__(self):
        self.hmd_controller = HMDController()
        self.touch_controller = TouchController()

    def get_hmd_controls(self, hmd_inputs, measures):
        return self.hmd_controller.compute_controls(hmd_inputs, measures)

    def get_touch_controls(self, touch_inputs):
        return self.touch_controller.compute_controls(touch_inputs)


if __name__ == '__main__':
    print("LQR sample")
    hmd_controller = HMDController()
    hmd_controller.simulator = hmd_controller.get_simulator(hmd_controller.model)
    x0 = np.array([[0], [0]])
    hmd_controller.simulator.x0 = x0
    #controller.lqr.set_initial_guess() # Use initial state to set the initial guess.
    for k in range(50):
        hmd_controller.lqr.set_setpoint(xss=np.array([[sin(k)], [cos(k)]]))
        u0 = hmd_controller.lqr.make_step(x0)
        y_next = hmd_controller.simulator.make_step(u0)
        x0 = y_next
    # Display the results
    rcParams['axes.grid'] = True
    rcParams['font.size'] = 18
    fig, ax, graphics = do_mpc.graphics.default_plot(hmd_controller.lqr.data, figsize=(16,9))
    graphics.plot_results()
    graphics.reset_axes()
    plt.show()
