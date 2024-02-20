from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
import math
import numpy as np

'''from mirobot_env import *
from stable_baselines3.common.env_checker import check_env
import numpy as np


env = MirobotEnv()
env.reset()
check_env(env)

env.reset()
print("ENV: ",env)

r_list = []

i = 0
while(i < 1000):
    a  = env.action_space.sample()
    print(a)
    obs, reward, terminated, truncated, info = env.step(a)
    print(obs, reward, terminated, truncated)
    r_list.insert(i, r)
    i = i+1
    print("[EnvTest] Reward:", env.reward)'''
    
#print("Average Reward: ", np.mean(r_list))
orientation_diff = [12, -12, 10]

odiffsum = sum([abs(od) for od in orientation_diff])
print("ODIFFSUM: ", odiffsum)