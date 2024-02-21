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
if not os.path.exists(models_dir):
    os.makedirs(models_dir)
if not os.path.exists(logdir):
    os.makedirs(logdir)

env = MirobotEnv()
env.reset()

policy_kwargs = dict(activation_fn= th.nn.ReLU, net_arch=dict(pi=[128, 128], vf=[128, 128]))

model = TRPO("MlpPolicy",
            gamma=0.995,
            batch_size=32,
            #learning_rate=0.04,
            env=env,
            verbose=1,
            policy_kwargs=policy_kwargs,
            tensorboard_log=logdir)
try:
    for i in range(1,EPISODES):
        model.learn(total_timesteps= TIMESTEPS, reset_num_timesteps= False, tb_log_name=MODELNAME)
        model.save(f"{models_dir}/{TIMESTEPS*i}")
except KeyboardInterrupt:
    print("[MirobotLearn] Keyboard Interrupt")
    env.reset()