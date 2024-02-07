from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
import math

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

'''
obs = [220, 150, 100, 120, -60, -85]
G = [220, -150, 100, 120, -60, -85]

for current, goal in zip(obs, G):
    #print(abs(current - goal) < 0.2)
    if abs(current - goal) > 0.2:
        print("return false")
print("return true")
'''
array1 = [265, -27.5, 80, 0, -90, 0]
array2 = [315.5, 41.25, 76.64, 179.5, -78.6, -103]

result = [round(x - y, 2) for x, y in zip(array1, array2)]
print(result)

array1 = [265, 0, 80, 0, -90, 0]
array2 = [314.6, 71.9, 76.8, 86.63, -90, 2.8]

result = [round(x - y, 2) for x, y in zip(array1, array2)]
print(result)

array1 = [265, 27.5, 80, 0, -90, 0]
array2 = [313.5, 98, 76.64, 179.5, -78.6, -90.2]

result = [round(x - y, 2) for x, y in zip(array1, array2)]
print(result)

array1 = [170, 122, 62, 90, 6.5, -90]
array2 = [198.5, 206.5, 58, 180, -26, -10.5]

result = [round(x - y, 2) for x, y in zip(array1, array2)]
print(result)

array1 = [195, 127, 62, 90, 6.5, -90]
array2 = [226.5, 210.85, 58, 180, -31.65, -17.81]

result = [round(x - y, 2) for x, y in zip(array1, array2)]
print(result)