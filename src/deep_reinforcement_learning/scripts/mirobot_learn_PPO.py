from mirobot_env import *
from stable_baselines3 import PPO
import os 
import datetime
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from stable_baselines3.common.policies import ActorCriticPolicy
### 
TIMESTEPS = 1000 # probably 100000
EPISODES = 1000000   # probably auch so 1000 
current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
MODELNAME = f"ReccurrentPPO_{current_time}"
###

models_dir = "drlsaves/models/"+MODELNAME
logdir = "drlsaves/rllogs"
print(models_dir) 

if not os.path.exists(models_dir):
    os.makedirs(models_dir)
if not os.path.exists(logdir):
    os.makedirs(logdir)

env = MirobotEnv()
env.reset()
print("[mirobot_env] environment: ", env)

n_actions = env.action_space.shape[-1]
action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

model = PPO(policy=ActorCriticPolicy, env=env, action_noise=action_noise, verbose = 1, tensorboard_log=logdir)

for i in range(1,EPISODES):
    model.learn(total_timesteps= TIMESTEPS, reset_num_timesteps= False, tb_log_name=MODELNAME)
    model.save(f"{models_dir}/{TIMESTEPS*i}")
