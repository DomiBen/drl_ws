from mirobot_env import *
from stable_baselines3.common.env_checker import check_env
import numpy as np

env = MirobotEnv()
check_env(env)

env.reset()
print("ENV: ",env)
'''
import math
n1 = math.nan
print(n1)
print(math.isnan(n1))
'''