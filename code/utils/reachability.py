# -*- coding: utf-8 -*-
"""
Created on Tue May 14 10:42:51 2024

@author: Edgar G.
"""

import copy
import torch
import random
import itertools
import numpy as np
import matplotlib.pyplot as plt
from utils.utils import vertices

def random_state(robot):
    return np.random.uniform(robot.min_state, robot.max_state)

def sample_reachable_set(robot, x0, total_samples: int, total_steps: int):
    final_states = []
    u_dim = robot.action_min.shape[1]
    for _ in range(total_samples):
        robot.reset_to(x0)
        controls = np.random.uniform(robot.action_min, robot.action_max, [total_steps, u_dim])
        for ui in controls:
            xF,_,_,_,_ = robot.step(ui) 
        final_states.append(xF)

    return final_states;

def set_from_vertices(robot, x0, total_steps: int):
    ctrls = vertices(robot.action_min, robot.action_max)[0];

    print(f"ctrls: {ctrls}")

    final_states = []
    for ui in ctrls:
        robot.reset_to(x0)
        print(ui)
        for _ in range(total_steps):
            xF,_,_,_,_ = robot.step(ui) 
        final_states.append(xF)
    return final_states

def state_to_str(state):
    s = ""
    for e in state:
        s += str(e) 
        s += " "
    return s;

def set_of_states_to_csv(set_of_states, filename: str):
    file = open(filename, 'w')

    for xi in set_of_states:
        xi_str = state_to_str(xi);
        xi_str += "\n";
        file.write(xi_str)
    file.close();

