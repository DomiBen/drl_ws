from mirobot_env import *
from stable_baselines3 import TD3
import os 
import datetime
from stable_baselines3.common.noise import OrnsteinUhlenbeckActionNoise
### 
TIMESTEPS = 500 # probably 10000
EPISODES = 1000000   # probably auch so 1000 
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
MODELNAME = f"TD3_{current_time}_OrnsteinUhlenbeckNoise0_2_lr0_00075_gamma0_999_target_policy_noise0_1_target_noise_clip0_25_delay3"
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
action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=0.2 * np.ones(n_actions))

model = TD3("MlpPolicy",
            env=env,
            action_noise=action_noise,
            #batch_size=64,
            learning_rate=0.00075,
            learning_starts=50,
            #tau=0.005,
            gamma=0.999,
            target_policy_noise=0.1,
            target_noise_clip=0.25,
            #buffer_size=256,
            policy_delay=3,
            verbose = 1,
            tensorboard_log=logdir)

try:
    for i in range(1,EPISODES):
        model.learn(total_timesteps= TIMESTEPS, reset_num_timesteps= False, tb_log_name=MODELNAME)
        model.save(f"{models_dir}/{TIMESTEPS*i}")
except KeyboardInterrupt:
    print("[MirobotLearn] Keyboard Interrupt")