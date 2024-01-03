from mirobot_env import *
from sb3_contrib import TRPO
import os 
import datetime
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from stable_baselines3.a2c.policies import ActorCriticPolicy
#from stable_baselines3.common.policies import MlpPolicy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.vec_env import VecNormalize
from stable_baselines3.common.policies import ActorCriticPolicy

### 
TIMESTEPS = 1000
EPISODES = 1000000
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
MODELNAME = f"TRPO_{current_time}_lr0_001_gamma0_995_targetkl0_04_batchsize20"
###

models_dir = "drlsaves/models/"+MODELNAME
logdir = "drlsaves/rllogs"

if not os.path.exists(models_dir):
    os.makedirs(models_dir)
if not os.path.exists(logdir):
    os.makedirs(logdir)

env = MirobotEnv()
obs, i = env.reset()
print("[mirobot_env] environment: ", env)

# Custom actor and critic network
class CustomPolicy(ActorCriticPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomPolicy, self).__init__(*args, **kwargs,
                                           net_arch=[dict(pi=[64, 64], vf=[64, 64])])

model = TRPO(policy=CustomPolicy,
            env=env,
            #batch_size=20,
            #learning_rate=0.001,
            #gamma=0.995,
            #target_kl=0.04,
            verbose=2,
            tensorboard_log=logdir)

for i in range(1, EPISODES):
    model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name=MODELNAME)
    model.save(f"{models_dir}/{TIMESTEPS*i}")
    
    '''obs, i = env.reset()
    for i in range(100):
        action, i = model.predict(obs, deterministic=True)
        obs, reward, term, trunc, i = env.step(action)
        if term or trunc:
            obs, info = env.reset()'''
