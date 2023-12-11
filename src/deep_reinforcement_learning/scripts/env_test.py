from mirobot_env import *
import numpy as np

env = MirobotEnv()
env.reset()
print(env)

t_list = []
f_list = []
i = 0
while(i < 2):
    a  = env.action_space.sample()
    print(a)
    a, b, c, d, f, t = env.step(a)
    t_list.insert(i, t)
    f_list.insert(i, f)
    i = i+1
    
print(env.score)

#print(t_list)
#print(f_list)
