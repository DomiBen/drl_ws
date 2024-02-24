from mirobot_env import *
from sb3_contrib import TRPO
import torch as th
import os 
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
env.reset()

model = TRPO.load("drlsaves/models/TRPO_custom_policy_2024_02_22_22_05_05_gamma_0995_batch_512_256_512NN/245000", env=env)

try:
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
                break #obs, info = env.reset()
            print("Action: ", action)
            print("Reward: ", reward)
            print("Terminated: ", terminated)
            print("Truncated: ", truncated)
            print("Info: ", info)
            print("Obs: ", obs)
            print("Episode: ", ep)
            print("Timestep: ", i)
            print("\n")
except KeyboardInterrupt:
    print("[Mirobot TRPO Execution] Keyboard Interrupt")
    env.reset()