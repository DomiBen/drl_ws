from mirobot_env import *
from stable_baselines3 import DDPG
import os 
import datetime
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from stable_baselines3.td3.policies import TD3Policy
### 
TIMESTEPS = 1000 # probably 100000
EPISODES = 1000000   # probably auch so 1000 
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
MODELNAME = f"DDPG_{current_time}_OrnsteinUhlenbeck_lr0_01_batchsize64_gamma1_5_buffer256"
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

model = DDPG(policy=TD3Policy,
             env=env,
             batch_size=64,
             learning_rate=0.01,
             gamma=1.5,
             buffer_size= 256,
             #tau=0.005,
             action_noise = action_noise,
             verbose = 1,
             tensorboard_log=logdir)

try:
    for i in range(1,EPISODES):
        model.learn(total_timesteps= TIMESTEPS, reset_num_timesteps= False, tb_log_name=MODELNAME)
        model.save(f"{models_dir}/{TIMESTEPS*i}")
except KeyboardInterrupt:
    print("[MirobotLearn] Keyboard Interrupt")
    env.reset()