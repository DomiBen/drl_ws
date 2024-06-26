from mirobot_env import *
from sb3_contrib import TRPO
import torch as th
import os 
import datetime

### Setting up parameters for the RL task ###
TIMESTEPS = 1000 
EPISODES = 10000
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
MODELNAME = f"New_Path_TRPO_custom_policy_{current_time}_gamma_0995_batch_512_256NN_256NN_peakForces"
models_dir = "drlsaves/models/"+MODELNAME
logdir = "drlsaves/rllogs/"
###
if not os.path.exists(models_dir):  # create the directory if it does not exist
    os.makedirs(models_dir)
if not os.path.exists(logdir):
    os.makedirs(logdir)

env = MirobotEnv()
env.reset()
# defining the policy network
policy_kwargs = dict(activation_fn= th.nn.ReLU, net_arch=dict(pi=[256, 256], vf=[256, 256]))

# in case you want to create a new model
model = TRPO("MlpPolicy",
            gamma=0.995,
            batch_size = 512,           # default 128
            #learning_rate=0.0005,       # default 0.001
            env=env,
            verbose=1,
            policy_kwargs=policy_kwargs,
            tensorboard_log=logdir)

# in case you want to continue training a model
#model = TRPO.load("/home/domi/drl_ws/drlsaves/models/New_Path_TRPO_custom_policy_2024_03_12_19_06_45_gamma_0995_batch_512_256NN_256NN_peakForces/54000", env=env)

try:
    for i in range(1,EPISODES):
        model.learn(total_timesteps= TIMESTEPS, reset_num_timesteps= False, tb_log_name=MODELNAME)
        model.save(f"{models_dir}/{TIMESTEPS*i}")
except KeyboardInterrupt:
    print("[MirobotLearn] Keyboard Interrupt")
    env.reset() 