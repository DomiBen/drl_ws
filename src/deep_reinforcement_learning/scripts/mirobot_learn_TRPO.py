from mirobot_env import *
from sb3_contrib import TRPO
import os 
import datetime
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from stable_baselines3.a2c.policies import ActorCriticPolicy
### 
TIMESTEPS = 1000 # probably 10000
EPISODES = 1000000   # probably auch so 1000 
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
MODELNAME = f"TRPO_{current_time}_OrnsteinUhlenbeckNoise"
###

models_dir = "drlsaves/models/"+MODELNAME
logdir = "drlsaves/rllogs"

if not os.path.exists(models_dir):
    os.makedirs(models_dir)
if not os.path.exists(logdir):
    os.makedirs(logdir)

env = MirobotEnv()
env.reset()
print("[mirobot_env] environment: ", env)

model = TRPO(policy=ActorCriticPolicy,
            env=env,
            #batch_size=64,
            #learning_rate=0.01,
            #gamma=1.5,
            #target_kl =0.05,
            verbose = 1,
            tensorboard_log=logdir)

for i in range(1,EPISODES):
    model.learn(total_timesteps= TIMESTEPS, reset_num_timesteps= False, tb_log_name=MODELNAME)
    model.save(f"{models_dir}/{TIMESTEPS*i}")
