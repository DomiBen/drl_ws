from mirobot_env import *
from sb3_contrib import TRPO
import os 
import datetime
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
### 
TIMESTEPS = 500 
EPISODES = 10000
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
MODELNAME = f"TRPO_discreteV2_{current_time}_gamma_09975"
models_dir = "drlsaves/models/"+MODELNAME
logdir = "drlsaves/rllogs/"
###

if not os.path.exists(models_dir):
    os.makedirs(models_dir)
if not os.path.exists(logdir):
    os.makedirs(logdir)

env = MirobotEnv()
env.reset()
print("[mirobot_env] environment: ", env)

model = TRPO("MlpPolicy",
            gamma=0.995,
            #n_steps=2048,
            env=env,
            verbose=1,
            tensorboard_log=logdir)

for i in range(1,EPISODES):
    model.learn(total_timesteps= TIMESTEPS, reset_num_timesteps= False, tb_log_name=MODELNAME)
    model.save(f"{models_dir}/{TIMESTEPS*i}")
