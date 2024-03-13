from mirobot_env import *
from sb3_contrib import TRPO
import torch as th
import os 
import datetime
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from sensor_logger_node import Logger, log_action
### Setting up parameters for the RL task ###
EPISODES = 1
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
MODELNAME = f"TRPO_allForces_gamma0995_batch512_256NN"

'''ALLE GENUTZTEN MODELLE HIER:'''
#model_dir = "drlsaves/models/New_Path_TRPO_custom_policy_2024_02_26_16_58_00_gamma_0995_batch_512_256NN_256NN_ScaledReward/500000"
model_dir = "/home/domi/drl_ws/drlsaves/models/New_Path_TRPO_custom_policy_2024_03_12_19_06_45_gamma_0995_batch_512_256NN_256NN_avgForces/54000"
#model_dir = "/home/domi/drl_ws/drlsaves/models/New_Path_TRPO_custom_policy_2024_03_12_02_16_21_gamma_0995_batch_512_256NN_256NN_avgForces/120000"
#model_dir = "/home/domi/drl_ws/drlsaves/models/New_Path_TRPO_custom_policy_2024_03_10_23_12_08_gamma_0995_batch_512_256NN_256NN_allForces/260000"
###
sensor_logger = Logger(MODELNAME)
env = MirobotEnv()
env.reset()

model = TRPO.load(model_dir, env=env)

try:
    #sensor_logger.record = True
    for ep in range(EPISODES):
        obs, info = env.reset()
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
        print("Reward: ", reward)
        print("Terminated: ", terminated)
        print("Truncated: ", truncated)
        print("Episode: ", ep)
        print("\n")
    sensor_logger.record = False
    
except KeyboardInterrupt:
    print("[Mirobot TRPO Execution] Keyboard Interrupt")
    env.reset()
    