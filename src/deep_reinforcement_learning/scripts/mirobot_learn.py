from mirobot_env import *
from stable_baselines3 import PPO
import os 

### 
TIMESTEPS = 100 # probably 10000
EPISODES = 10   # probably auch so 1000 
MODELNAME = "PPO"
###

models_dir = "/drl_ws/src/deep_reinforcement_learning/models/"+MODELNAME
logdir = "/drl_ws/src/deep_reinforcement_learning/rllogs"
print(models_dir) 

if not os.path.exists(models_dir):
    os.makedirs(models_dir)
if not os.path.exists(logdir):
    os.makedirs(logdir)

env = MirobotEnv()
env.reset()
print("[mirobot_env] environment: ", env)

model = PPO("MlpPolicy", env, verbose = 1, tensorboard_log=logdir)

for i in range(1,EPISODES):
    model.learn(total_timesteps= TIMESTEPS, reset_num_timesteps= False, tb_log_name=MODELNAME)
    model.save(f"{models_dir}/{TIMESTEPS*i}")
