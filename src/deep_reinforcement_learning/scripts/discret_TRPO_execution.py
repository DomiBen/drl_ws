#!/usr/bin/env python
# =============================================================================
# Created By  : Dominik Benchert
# 
# Last Update : April 2024
# License     : BSD-3
# =============================================================================
"""
This script is used to execute the TRPO model on the Mirobot
"""
from mirobot_env import *
from sb3_contrib import TRPO
import datetime
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from sensor_logger_node import Logger, log_action

### Setting up parameters for the RL task ###
EPISODES = 1
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
MODELNAME = f"TRPO_peakForces"
### Model to be executed
model_dir = "/home/domi/drl_ws/drlsaves/models/New_Path_TRPO_custom_policy_2024_03_02_11_03_42_gamma_0995_batch_512_256NN_256NN_OldReward_PeakForces/500000"
#model_dir = "/home/domi/drl_ws/drlsaves/models/New_Path_TRPO_custom_policy_2024_03_12_02_16_21_gamma_0995_batch_512_256NN_256NN_avgForces/120000"
#model_dir = "/home/domi/drl_ws/drlsaves/models/New_Path_TRPO_custom_policy_2024_03_10_23_12_08_gamma_0995_batch_512_256NN_256NN_allForces/260000"
###

sensor_logger = Logger(MODELNAME)
env = MirobotEnv()
env.reset()
model = TRPO.load(model_dir, env=env)

try:
    for ep in range(EPISODES):
        obs, info = env.reset()
        #sensor_logger.record = True
        for i in range(1, 1000):
            action, _states = model.predict(obs, deterministic=True)
            log_action(action)
            obs, reward, terminated, truncated, info = env.step(action)
            if terminated:
                print(f"Episode {ep} finished after {i} timesteps")
                break
            if truncated:
                print(f"Episode {ep} truncated after {i} timesteps")
                break 
    
except KeyboardInterrupt:
    print("[Mirobot TRPO Execution] Keyboard Interrupt")
    env.reset()
    