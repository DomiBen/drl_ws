from mirobot_env import *
from stable_baselines3 import PPO
import os 
import datetime
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from stable_baselines3.common.policies import ActorCriticPolicy
### 
TIMESTEPS = 1000 # probably 100000
EPISODES = 1000000   # probably auch so 1000 
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
MODELNAME = f"PPO_{current_time}_lr0_001_entcoef0_075_gamma0_995"
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
#action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

model = PPO('MlpPolicy', 
            env=env, 
            #clip_range=0.2,
            #n_steps=128,
            learning_rate=0.001,
            ent_coef=0.075,
            #batch_size= 64,
            gamma= 0.995,
            verbose = 1,
            tensorboard_log=logdir)
try:
    for i in range(1,EPISODES):
        model.learn(total_timesteps = TIMESTEPS, reset_num_timesteps= False, tb_log_name=MODELNAME)
        model.save(f"{models_dir}/{TIMESTEPS*i}")
except KeyboardInterrupt:
    print("[MirobotLearn] Keyboard Interrupt")
    env.reset()
