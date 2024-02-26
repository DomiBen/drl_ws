from mirobot_env import *
from sb3_contrib import TRPO
import torch as th
import os 
import csv
import datetime
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

### Setting up parameters for the RL task ###
TIMESTEPS = 500 
EPISODES = 10000
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
MODELNAME = f"TRPO_custom_policy_{current_time}_gamma_0995_batch_32"
models_dir = "drlsaves/models/"+MODELNAME
logdir = "drlsaves/rllogs/"
###

env = MirobotEnv()
model = TRPO.load("/home/dominik/drl_ws/drlsaves/models/TRPO_custom_policy_2024_02_24_14_10_39_gamma_0995_batch_512_256NN_256NN/300000", env=env)

logfile = "/home/dominik/drl_ws/src/sensor_logger/logfiles/action_log.csv"
if not os.path.exists(logfile):
    os.makedirs(os.path.dirname(logfile), exist_ok=True)

try:
    #start recording sensor data
    obs, info = env.reset()
    for i in range(1, 1000):
        action, _states = model.predict(obs, deterministic=True)
        # Open the file in append mode
        with open(logfile, mode='a', newline='') as file:
            # Create a CSV writer object
            writer = csv.writer(file)
            writer.writerow(action)

        obs, reward, terminated, truncated, info = env.step(action)
        
        if terminated:
            print(f"Episode finished after {i} timesteps")
            break
        if truncated:
            print(f"Episode truncated after {i} timesteps")
            break #obs, info = env.reset()
    #stop recording sensor data
        
except KeyboardInterrupt:
    print("[Mirobot TRPO Execution] Keyboard Interrupt")
    env.reset()