from mirobot_env import *
from sb3_contrib import TRPO
import torch as th
import os 
import csv
import datetime
from sensor_logger_node import Logger

### Setting up parameters for the RL task ###
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
MODELNAME = f"TRPO_gamma_0995_batch_512_256NN_256NN"
EPISODES = 100
###
model_dir_1 = "/home/domi/drl_ws/drlsaves/models/New_Path_TRPO_custom_policy_2024_03_02_11_03_42_gamma_0995_batch_512_256NN_256NN_OldReward_PeakForces/500000"
model_dir_2 = "/home/domi/drl_ws/drlsaves/models/New_Path_TRPO_custom_policy_2024_03_12_02_16_21_gamma_0995_batch_512_256NN_256NN_avgForces/120000"
model_dir_3 = "/home/domi/drl_ws/drlsaves/models/New_Path_TRPO_custom_policy_2024_03_10_23_12_08_gamma_0995_batch_512_256NN_256NN_allForces/260000"
###

env = MirobotEnv()
env.reset()

peak_logger = Logger("LongtermTest_peakForces")
model = TRPO.load(model_dir_1, env=env)
for ep in range(EPISODES):
    obs, info = env.reset()
    peak_logger.record = True
    for i in range(1, 1000):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        if terminated:
            print(f"Episode {ep} finished after {i} timesteps")
            break
        if truncated:
            print(f"Episode {ep} truncated after {i} timesteps")
            break 
    peak_logger.record = False
    
avg_logger = Logger("LongtermTest_avgForces")
model = TRPO.load(model_dir_2, env=env)
for ep in range(EPISODES):
    obs, info = env.reset()
    avg_logger.record = True
    for i in range(1, 1000):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        if terminated:
            print(f"Episode {ep} finished after {i} timesteps")
            break
        if truncated:
            print(f"Episode {ep} truncated after {i} timesteps")
            break
    avg_logger.record = False
    
all_logger = Logger("LongtermTest_allForces")
model = TRPO.load(model_dir_3, env=env)
for ep in range(EPISODES):
    obs, info = env.reset()
    all_logger.record = True
    for i in range(1, 1000):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        if terminated:
            print(f"Episode {ep} finished after {i} timesteps")
            break
        if truncated:
            print(f"Episode {ep} truncated after {i} timesteps")
            break 
    all_logger.record = False
    
reference_logger = Logger("LongtermTest_refForces")
model = TRPO.load(model_dir_3, env=env)
for ep in range(EPISODES):
    obs, info = env.reset()
    reference_logger.record = True
    mirobot.moveToAbsolutePosition([40, 42, 23.5, -42.5, -161.5, -80])
    reference_logger.record = False