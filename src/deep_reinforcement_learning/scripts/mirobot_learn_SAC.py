from mirobot_env import *
from stable_baselines3 import SAC
import os
import datetime
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from stable_baselines3.sac.policies import SACPolicy
### 
TIMESTEPS = 1000 # probably 100000
EPISODES = 1000000   # probably auch so 1000 
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
MODELNAME = f"SAC_{current_time}_OrnsteinUhlenbeckNoise_lr001_batchsize64_gamma15_buffer256_entcoef005"
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

n_actions = env.action_space.shape[-1]
action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))


model = SAC(policy=SACPolicy,
            env=env,
            action_noise=action_noise,
            learning_rate=0.01,
            gamma=1.5,
            buffer_size=256,
            ent_coef=0.05,
            verbose = 1,
            tensorboard_log=logdir)

for i in range(1,EPISODES):
    model.learn(total_timesteps= TIMESTEPS, reset_num_timesteps= False, tb_log_name=MODELNAME)
    model.save(f"{models_dir}/{TIMESTEPS*i}")
