from mirobot_env import *
from stable_baselines3.common.env_checker import check_env
import numpy as np

env = MirobotEnv()

#check_env(env)

env.reset()
print("ENV: ",env)

r_list = []

i = 0
while(i < 1000):
    a  = env.action_space.sample()
    print(a)
    obs, reward, terminated, truncated, info = env.step(a)
    print(obs, reward, terminated, truncated)
    r_list.insert(i, r)
    i = i+1
    print("[EnvTest] Reward:", env.reward)
    
#print("Average Reward: ", np.mean(r_list))
