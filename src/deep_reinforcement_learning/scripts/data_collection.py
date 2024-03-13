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
model_dir_1 = "drlsaves/models/New_Path_TRPO_custom_policy_2024_02_26_16_58_00_gamma_0995_batch_512_256NN_256NN_ScaledReward/440000"
model_dir_2 = "/home/domi/drl_ws/drlsaves/models/New_Path_TRPO_custom_policy_2024_03_12_02_16_21_gamma_0995_batch_512_256NN_256NN_avgForces/120000"
model_dir_3 = "/home/domi/drl_ws/drlsaves/models/New_Path_TRPO_custom_policy_2024_03_10_23_12_08_gamma_0995_batch_512_256NN_256NN_allForces/260000"
###

env = MirobotEnv()
env.reset()

peak_logger = Logger("LongtermTest_peakForces")
model = TRPO.load(model_dir_1, env=env)
peak_logger.record = True
for ep in range(EPISODES):
    obs, info = env.reset()
    for i in range(1, 1000):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        if terminated:
            print(f"Episode {ep} finished after {i} timesteps")
            break
        if truncated:
            print(f"Episode {ep} truncated after {i} timesteps")
            break 
    print("Reward: ", reward)
    print("Terminated: ", terminated)
    print("Truncated: ", truncated)
    print("Episode: ", ep)
    print("\n")
peak_logger.record = False
    
avg_logger = Logger("LongtermTest_avgForces")
model = TRPO.load(model_dir_2, env=env)
avg_logger.record = True
for ep in range(EPISODES):
    obs, info = env.reset()
    for i in range(1, 1000):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        if terminated:
            print(f"Episode {ep} finished after {i} timesteps")
            break
        if truncated:
            print(f"Episode {ep} truncated after {i} timesteps")
            break 
    print("Reward: ", reward)
    print("Terminated: ", terminated)
    print("Truncated: ", truncated)
    print("Episode: ", ep)
    print("\n")
avg_logger.record = False
    
all_logger = Logger("LongtermTest_allForces")
model = TRPO.load(model_dir_3, env=env)
all_logger.record = True
for ep in range(EPISODES):
    obs, info = env.reset()
    for i in range(1, 1000):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        if terminated:
            print(f"Episode {ep} finished after {i} timesteps")
            break
        if truncated:
            print(f"Episode {ep} truncated after {i} timesteps")
            break 
        print("Reward: ", reward)
        print("Terminated: ", terminated)
        print("Truncated: ", truncated)
        print("Episode: ", ep)
        print("\n")
all_logger.record = False