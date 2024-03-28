#!/usr/bin/env python
# =============================================================================
# Created By  : Dominik Benchert
# 
# Last Update : April 2024
# License     : BSD-3
# =============================================================================
"""
This script is used to check the environment of the MirobotEnv class
"""

from mirobot_env import *
from stable_baselines3.common.env_checker import check_env
import numpy as np

env = MirobotEnv()
check_env(env)
env.reset()
print("ENV: ",env)

# testing the environment with 1000 random actions
for i in range(1, 1000):
    a  = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(a)
    print("[EnvTest] Reward:", env.reward)
    