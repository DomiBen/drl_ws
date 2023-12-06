import gymnasium as gym
import numpy as np
import math
from gymnasium import spaces
from time import sleep
from tf.transformations import euler_from_matrix , translation_from_matrix
from pykdl_utils.kdl_kinematics import KDLKinematics
from urdf_parser_py.urdf import URDF
from MirobotClient import *
import random as r

min_angles_deg = [-110, -35, -120, -180, -200, -360]
min_angles_rad = [i*math.pi/180 for i in min_angles_deg]
max_angles_deg = [160, 70, 60, 180, 30, 360]
max_angles_rad = [i*math.pi/180 for i in max_angles_deg]

mirobot = MirobotClient()
print("passed")
#load robot model from ros poarameter server
robot = URDF.from_parameter_server()
# creating KDLKinematiocs class
kdl_kin = KDLKinematics(robot, "base_link", "link6")

N_DISCRETE_ACTIONS = 2

class MirobotEnv(gym.Env):
    """Custom Environment that follows gym interface."""
    def __init__(self):
        super(MirobotEnv, self).__init__()
        # Define action and observation space -> must be gym space object 
        self.action_space = spaces.Box(low=np.append(min_angles_deg, [0]), 
                                       high=np.append(max_angles_deg, [2000]), dtype=np.float32)
        # Observation Space: all Values between the minimum and maximum angles Are possible; Box with diffent sized Vectors
        self.observation_space = spaces.Box(low=np.array(min_angles_deg, dtype=np.float32),
                                            high=np.array(max_angles_deg, dtype=np.float32), dtype=np.float32)
        while(mirobot.current_pose == None): 
            sleep(1)

    def step(self, action):
        print("[MirbotEnv] beginning step")
        mirobot.reset_ft_record()
        mirobot.executeAction(action)
        # observe outcome and check if goal is reached
        self.observation = mirobot.getObservation() #returns np.array([cart_x, cart_y, cart_z, euler_r, euler_p, euler_y]) 
        if self.observation is self.cart_goal:
            self.done = True
        else: 
            self.done = False
        # creating reward -> NOT FINISHED needs more feature engineering
        self.reward = self.getReward()
        info = {}
        return self.observation, self.reward, info, self.done, mirobot.average_force, mirobot.average_torque #terminated, truncated

    def reset(self):
        # reset initiates Environment variables
        self.done = False
        self.score = 0
        # generate new goal
        self.cart_goal = np.array(self.generateGoal(), dtype=np.float32) # random values for x, y, z, r, p, y
        pose_diff = [g-c for g, c in zip(self.cart_goal, mirobot.current_pose)]
        self.previous_distance = math.sqrt(sum([pow(x,2) for x in pose_diff[:3]]))
        self.previous_orientation_diff = math.sqrt(sum([pow(x,2) for x in pose_diff[3:]])) # euclidean distance -> TODO: check if other solution could be smart
        #observation
        self.observation = mirobot.getObservation() #returns np.array([cart_x, cart_y, cart_z, euler_r, euler_p, euler_y])
        mirobot.reset_ft_record()
        return self.observation #, info
    
    def generateGoal(self):
        #first pick random values for the joint angles, between theire minumum and maximum constraints
        rand_joint_states = [r.uniform(min_angles_rad[0], max_angles_rad[0]), r.uniform(min_angles_rad[1], max_angles_rad[1]), 
                             r.uniform(min_angles_rad[2], max_angles_rad[2]), r.uniform(min_angles_rad[3], max_angles_rad[3]), 
                             r.uniform(min_angles_rad[4], max_angles_rad[4]), r.uniform(min_angles_rad[5], max_angles_rad[5])]
        # then convert them to cartesian coordinates -> therefore we won't generate unreachable points
        pose = kdl_kin.forward(rand_joint_states)
        linear = translation_from_matrix(pose)
        euler = euler_from_matrix(pose)
        return [linear[0]*1000, linear[1]*1000, linear[2]*1000, euler[0]*180/math.pi, euler[1]*180/math.pi, euler[2]*180/math.pi]
    
    def getReward(self): 
        pose_diff = [g-c for g, c in zip(self.cart_goal, mirobot.current_pose)]
        distance = math.sqrt(sum([pow(x,2) for x in pose_diff[:3]]))
        distance_change = self.previous_distance - distance
        self.previous_distance = distance
        orientation_diff = math.sqrt(sum([pow(x,2) for x in pose_diff[3:]])) # euclidean distance -> TODO: check if other solution could be smart
        orientation_change = self.previous_orientation_diff - orientation_diff
        self.previous_orientation_diff = orientation_diff
        force = mirobot.average_force * 10
        torque = mirobot.average_torque * 200
        print('[MirobotEnv] [Reward]:', distance_change, orientation_change, force, torque)
        print('[MirobotEnv] [Reward]:', distance_change + orientation_change/distance - force - torque)
        if distance > 10.0:
            return  distance_change + orientation_change/distance - force - torque
        return  distance_change + orientation_change - force - torque 
