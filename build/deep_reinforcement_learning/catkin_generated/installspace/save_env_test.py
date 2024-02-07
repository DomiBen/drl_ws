from mirobot_env import *
import numpy as np

env = MirobotEnv()
env.reset()
print(env)

t_list = []
f_list = []
i = 0
a =  [90, 10, 20, 5, 12, 180, 500]
print(a)
a, b, c, d, f, t = env.step(a)
t_list.insert(i, t)
f_list.insert(i, f)

a =  [0, 0, 0, 0, 90, 0, 0]
print(a)
a, b, c, d, f, t = env.step(a)
t_list.insert(i, t)
f_list.insert(i, f)
    
print(env.score)

#print(t_list)
#print(f_list)
