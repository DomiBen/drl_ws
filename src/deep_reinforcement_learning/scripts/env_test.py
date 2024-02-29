from mirobot_env import *
from sb3_contrib import TRPO
import torch as th
import os 
import csv
import datetime
from sensor_logger_node import Logger

### Setting up parameters for the RL task ###
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
MODELNAME = f"TRPO_gamma_0995_batch_512_256NN_256NN"
models_dir = "/home/domi/drl_ws/drlsaves/models/TRPO_custom_policy_2024_02_24_14_10_39_gamma_0995_batch_512_256NN_256NN/320000"
###

env = MirobotEnv()
sensor_logger = Logger(MODELNAME)
model = TRPO.load(models_dir, env=env)

logfile = "/home/domi/drl_ws/src/sensor_logger/logfiles/action_log.csv"
if not os.path.exists(logfile):
    os.makedirs(os.path.dirname(logfile), exist_ok=True)

try:
    sensor_logger.record = True
    obs, info = env.reset()
    for i in range(1, 1000):
        action, _states = model.predict(obs, deterministic=True)
        # Open the file in append mode
        with open(logfile, mode='a', newline='') as file:
            # Create a CSV writer object
            writer = csv.writer(file)
            writer.writerow(action)
        obs, reward, terminated, truncated, info = env.step(action)
        
        if terminated:
            print(f"Episode finished after {i} timesteps")
            sensor_logger.record = False
            break
        if truncated:
            print(f"Episode truncated after {i} timesteps")
            sensor_logger.record = False
            break #obs, info = env.reset()

except KeyboardInterrupt:
    print("[Mirobot TRPO Execution] Keyboard Interrupt")
    env.reset()