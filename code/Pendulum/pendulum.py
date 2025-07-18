# -*- coding: utf-8 -*-
"""
Created on Thu Dec  5 14:09:01 2024

@author: Edgar G.

"""

import torch
import numpy as np

class Pendulum():
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
        
        self.name = "Pendulum"
        self.state_size = 2
        self.action_size = 1
        self.torque_max = 0.6371781908344007
        self.action_min = np.array([[ -self.torque_max ]])
        self.action_max = np.array([[  self.torque_max ]])
        
        ### Fundamental quad parameters
        self.gravity = 9.81 # gravity (m/s^2)
        
        self.length = 0.5;
        self.friction = 0.1;
        self.mass = 0.15;
        
        self.dt = dt # time step

        self.state = np.array([0, 0])

        self.min_state = np.array([-np.pi, -2.0 * np.pi])
        self.max_state = np.array([+np.pi, +2.0 * np.pi])
    
    @property
    def theta(self):
        return self.state[0]
    
    @property
    def thetadot(self):
        return self.state[1];

    def compute_inertia(self):
        return self.mass * self.length * self.length;
        
    def reset(self):
        self.state = np.array([0, 0])
        return self.state.copy()
        
    
    def reset_to(self, state):
        self.state = state.copy()
        return state

        
    def step(self, u):
        inertia = self.compute_inertia();

        thetadotdot = self.gravity / self.length * np.sin(self.theta) + u / inertia;
        thetadotdot -= self.friction / inertia * self.thetadot;

        # print(f"thetadot: {self.thetadot} thetadotdot: {thetadotdot}")
        xd = np.array([self.thetadot, thetadotdot[0]]);
        # self.state = self.state +  xd * self.dt # one time step forward
         # RK45(fun, t0, y0, t_bound);
        reward = 1; # ToDo: Define a reward

        # This is ridiculous, stepping an ode should be only that
        return self.state.copy(), reward, False, False, None 

    # Function called by the projectors
    # def pos_from_vel(self, S_t, vel_t_dt):
    #     """
    #     Calculates the next state's position using explicit Euler integrator
    #     and quaternion formula, does NOT need to know the dynamics.
        
    #     Arguments:
    #         - S_t : current state torch.tensor (17,)
    #         - vel_t_dt : (unused) next state's velocity torch.tensor (10,)
    #     Returns:
    #         - x_t_dt : next state's position torch.tensor (7,)
    #     """
    #     x_t_dt = S_t[:7].clone() # copy the current position
    #     q0 =    S_t[3]
    #     q1 =    S_t[4]
    #     q2 =    S_t[5]
    #     q3 =    S_t[6]
    #     xdot =  S_t[7]
    #     ydot =  S_t[8]
    #     zdot =  S_t[9]
    #     p =     S_t[10]
    #     q =     S_t[11]
    #     r =     S_t[12]
        
    #     x_t_dt += self.dt*torch.FloatTensor([xdot, ydot, zdot,
    #                                -0.5*p*q1 - 0.5*q*q2 - 0.5*q3*r,
    #                                 0.5*p*q0 - 0.5*q*q3 + 0.5*q2*r,
    #                                 0.5*p*q3 + 0.5*q*q0 - 0.5*q1*r,
    #                                -0.5*p*q2 + 0.5*q*q1 + 0.5*q0*r]).to(S_t.device)

    #     return x_t_dt

    # Plotting functions
    # def plot_traj(self, Traj, title:str = ""):
    #     """Plots the xy trajectory of the Quadcopter."""
    #     plot_traj(self, Traj, title)

    
    # def traj_comparison(self, traj_1, label_1, traj_2, label_2, title:str = "",
    #                     traj_3=None, label_3=None, traj_4=None, label_4=None,
    #                     legend_loc='best'):
        """
        Compares up to 4 xy trajectories of the Quadcopter
        Arguments:
            - traj_1 : first trajectory of shape (H, 17)
            - label_1 : corresponding label to display
            - traj_2 : first trajectory of shape (H, 17)
            - label_2 : corresponding label to display
            - title: optional title of the plot
            - traj_3 : optional third trajectory of shape (H, 17)
            - label_3 : optional corresponding label to display
            - traj_4 : optional fourth trajectory of shape (H, 17)
            - label_4 : optional corresponding label to display
            - legend_loc : optional location of the legend
        """
        # traj_comparison(self, traj_1, label_1, traj_2, label_2, title,
        #                 traj_3, label_3, traj_4, label_4, legend_loc)

         

