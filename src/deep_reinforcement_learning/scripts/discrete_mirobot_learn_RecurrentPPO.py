from mirobot_env import *
from sb3_contrib import RecurrentPPO
import os 
import datetime
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
### Setting up parameters for the RL task ### 
TIMESTEPS = 1000
EPISODES = 1000000 
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
MODELNAME = f"RecurrentPPO_discreteV2_{current_time}_gamma0_995"
###
models_dir = "drlsaves/models/"+MODELNAME
logdir = "drlsaves/rllogs"

if not os.path.exists(models_dir):
    os.makedirs(models_dir)
if not os.path.exists(logdir):
    os.makedirs(logdir)

env = MirobotEnv()
env.reset()
n_actions = env.action_space.shape[-1]
action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

model = RecurrentPPO('MlpLstmPolicy', 
            env=env, 
            #clip_range=0.2,
            #n_steps=128,
            #clip_range=0.5,
            #learning_rate=0.001,
            #ent_coef=0.3,
            #batch_size= 32,
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
