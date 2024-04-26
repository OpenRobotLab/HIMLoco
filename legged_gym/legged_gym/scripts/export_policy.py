from legged_gym import LEGGED_GYM_ROOT_DIR
import os

import isaacgym
from legged_gym.envs import *
from legged_gym.utils import  get_args, export_policy_as_jit, task_registry, Logger
from rsl_rl.modules import ActorCriticLSTM, ActorCritic

import numpy as np
import torch


def export(args):
    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)
    ac = ActorCritic(num_actor_obs=45*6, num_critic_obs=238, num_one_step_obs=45, num_actions=12).to('cpu')
    state_dict = torch.load("/home/longjf/legged_gym/logs/rough_go1/Sep21_20-18-45_/model_460.pt", map_location='cpu')
    ac.load_state_dict(state_dict['model_state_dict'])

    path = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', train_cfg.runner.experiment_name, 'exported', 'policies')
    export_policy_as_jit(ac, path)
    print('Exported policy as jit script to: ', path)
    
if __name__ == '__main__':
    args = get_args()
    export(args)