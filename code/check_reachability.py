# -*- coding: utf-8 -*-
"""
Created on Thu Feb 13 15:46:22 2025

@author: Jean-Baptiste Bouvier

Main script to load and evaluate a pre-trained Diffusion Transfomer
"""

import torch
import numpy as np
import argparse


from utils.loaders import make_env, load_proj
from utils.utils import set_seed, open_loop, vertices
from utils.inverse_dynamics import InverseDynamics
from utils.reachability import *
from DiT.ODE import ODE
from DiT.planner import Planner



if __name__ == "__main__":
    argparse = argparse.ArgumentParser()
    # argparse.add_argument('-f', '--file', help='Gait file', required=True)
    argparse.add_argument('-n', '--name', help='Name', required=True)
    argparse.add_argument('-s', '--total_samples', help='Name', required=True)
    argparse.add_argument('--steps', help='Start State', required=True)
    argparse.add_argument('-x', '--start_state', help='Start State', required=True, nargs='+')
    args = argparse.parse_args()
    

    robot_name = args.name # name of the environment in ["Hopper", "Walker", "HalfCheetah", "Quadcopter", "GO1", "GO2"]
    mode = None
    robot, _, _, _ = make_env(robot_name, mode)


    max_ctrls = vertices(robot.action_min, robot.action_max);
    print(f"max_ctrls: {max_ctrls}")

    total_steps = int(args.steps)
    # x0 = random_state(robot);
    x0 = np.array([0] * len(args.start_state))
    for i in range(len(args.start_state)):
        x0[i] = float(args.start_state[i])
    
    print(f"x0: {x0}")
    
    approx_set = set_from_vertices(robot, x0, total_steps)

    samples = int(args.total_samples)
    reached_set = sample_reachable_set(robot, x0, samples, total_steps)

    set_of_states_to_csv(reached_set, robot_name + "_reached_set.txt");
    set_of_states_to_csv(approx_set, robot_name + "_approx_set.txt");

