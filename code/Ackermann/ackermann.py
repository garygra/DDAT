# -*- coding: utf-8 -*-
"""

@author: Edgar G.

"""

import torch
import numpy as np

import utils.integrators as Integrators
#import euler, rk45

class Ackermann():
    """
    Simple pendulum: a 2-dim toy system 
    ## Control Space... because this is robotics
    | Num | Control  | Min                  | Max                | Name | Unit |
    | --- | ---------| ---                  | ---                | ---- | ---- |
    | 0   | Torque   | -0.6371781908344007  | 0.6371781908344007 |   u  |  N   |

    ## State Space
    | Num | State                            | Min   | Max  | Name | Unit  |
    | --- | --------------------------------------   | ---- | ---  | ---- | ----- |
    | 0   | theta - angle                    | -pi   | pi   |  th  |  m    |
    | 1   | \\dot{theta} - angular velocity   | -2*pi | 2*pi |  dth |  m    |
    """
    
    
    def __init__(self, dt: float = 0.01):
        
        self.name = "Ackermann"
        self.state_size = 3
        self.action_size = 2

        max_delta_deg = 60;
        max_delta_rad = max_delta_deg * np.pi / 180.0;
        self.action_min = np.array([[ -max_delta_rad, 0 ]])
        self.action_max = np.array([[  max_delta_rad,  1 ]])
        
        self.L = 1.0;

        self.dt = dt # time step

        self.state = np.array([0.0, 0.0, 0.0])
        self.control = np.array([0.0, 0.0])

        self.min_state = np.array([-100, -100, -np.pi])
        self.max_state = np.array([ 100,  100, +np.pi])
        
        self.integrator = "RK45"

    @property
    def x(self):
        return self.state[0]
    
    @property
    def y(self):
        return self.state[1];

    @property
    def theta(self):
        return self.state[2];

    # @theta.getter
    # def theta(self):
    #     return self.state[2];
    
    @theta.setter
    def theta(self, th):
        self.state[2] = th;

    @property
    def V(self):
        return self.control[1]

    @property
    def Steer(self):
        return self.control[0]

    # def V(self, u):
    #     return u[1]
    
    # def Steer(self, u):
    #     return u[0]

    @theta.setter
    def Control(self, u):
        self.control = u.copy()

    def reset(self):
        self.state = np.array([0, 0, 0.0])
        return self.state.copy()
        
    def reset_to(self, state):
        self.state = state.copy()
        return state

    def xdot_rhs(self, x, u):
        self.reset_to(x)

        self.Control = u

        V = self.V
        steer = self.Steer
        
        x_dot = V * np.cos(self.theta);
        y_dot = V * np.sin(self.theta);
        theta_dot = (V / self.L) * np.tan(steer);


        xd = np.array([x_dot, y_dot, theta_dot]);
        return xd;


    def integrate(self):
        if self.integrator == "Euler":
            # xd = self.xdot_rhs(self.state, self.control)
            # from utils.integrators import euler, rk45
            self.state = Integrators.euler(self.xdot_rhs, self.state, self.control, self.dt)
            # self.state = self.state + xd * self.dt

        if self.integrator == "RK45":
            self.state = Integrators.rk45(self.xdot_rhs, self.state, self.control, self.dt)

            # self.state = RK45(fun, t0, y0, t_bound, max_step=inf, rtol=0.001, atol=1e-06, vectorized=False);

    def step(self, u):
        self.Control = u;
        # print(f"u {u}")
        self.integrate();

        # self.state = self.state + xd * self.dt # one time step forward

        r21 = np.sin(self.theta);
        r11 = np.cos(self.theta);
        self.theta = np.arctan2(r21, r11);

        reward = 1; # ToDo: Define a reward

        # This is ridiculous, stepping an ode should be only that
        return self.state.copy(), reward, False, False, None 


