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
from sklearn import preprocessing

min_angles_deg = [-110, -35, -120, -180, -200, -360]
min_angles_rad = [i*math.pi/180 for i in min_angles_deg]
max_angles_deg = [160, 70, 60, 180, 30, 360]
max_angles_rad = [i*math.pi/180 for i in max_angles_deg]

mirobot = MirobotClient()
#load robot model from ros poarameter server and create KDLKinematiocs class
robot = URDF.from_parameter_server()
kdl_kin = KDLKinematics(robot, "base_link", "link6")

N_DISCRETE_ACTIONS = 2

# Custom Environment that follows gym interface
class MirobotEnv(gym.Env):
    def __init__(self):
        super(MirobotEnv, self).__init__()
        # Define action and observation space -> must be gym space object 
        self.action_space = spaces.Box(low=np.append(min_angles_deg, [0]), 
                                       high=np.append(max_angles_deg, [2000]), dtype=np.float32)
        # Observation Space: all Values between the minimum and maximum angles Are possible; Box with diffent sized Vectors
        self.observation_space = spaces.Box(high=np.array([326, 246, 429, 360, 360, 360], dtype=np.float32),
                                            low=np.array([-230, -246, -31, -360, -360, -360], dtype=np.float32), dtype=np.float32)
        while(mirobot.current_pose == None): 
            sleep(1)

    def step(self, action):
        print("[MirobotEnv] [step]      beginning step")
        mirobot.reset_ft_record()
        mirobot.executeAction(action)
        # observe outcome and check if goal is reached
        observation = mirobot.getObservation() #returns np.array([cart_x, cart_y, cart_z, euler_r, euler_p, euler_y])
        # check if terminated 
        if self.goalReached(observation):
            self.terminated = True
        else: 
            self.terminated = False
        # creating reward
        self.reward = self.getReward()
        self.score = self.score + self.reward
        # check if Truncated
        if observation[2] < 1:
            self.truncated = True
        else: 
            self.truncated = False
        info = {}
        return observation, self.reward, self.terminated, self.truncated, info

    def reset(self, seed=None, options=None):
        # reset initiates Environment variables
        self.terminated = False
        self.truncated = False
        self.score = 0
        # generate new goal
        self.cart_goal = np.array(self.generateGoal(), dtype=np.float32) # random values for x, y, z, r, p, y
        pose_diff = [g-c for g, c in zip(self.cart_goal, mirobot.current_pose)]
        self.previous_distance = math.sqrt(sum([pow(x,2) for x in pose_diff[:3]]))
        self.previous_orientation_diff = math.sqrt(sum([pow(x,2) for x in pose_diff[3:]])) # euclidean distance -> TODO: check if other solution could be smart
        #observation
        observation = mirobot.getObservation() #returns np.array([cart_x, cart_y, cart_z, euler_r, euler_p, euler_y])
        mirobot.reset_ft_record()
        info = {}
        return observation, info
    
   #def close(): 
    
    def generateGoal(self):
        #first pick random values for the joint angles, between theire minumum and maximum constraints
        rand_joint_states = [r.uniform(min_angles_rad[0], max_angles_rad[0]), r.uniform(min_angles_rad[1], max_angles_rad[1]), 
                             r.uniform(min_angles_rad[2], max_angles_rad[2]), r.uniform(min_angles_rad[3], max_angles_rad[3]), 
                             r.uniform(min_angles_rad[4], max_angles_rad[4]), r.uniform(min_angles_rad[5], max_angles_rad[5])]
        # then convert them to cartesian coordinates -> therefore we won't generate unreachable points
        pose = kdl_kin.forward(rand_joint_states)
        linear = translation_from_matrix(pose)
        euler = euler_from_matrix(pose)
        goal = [linear[0]*1000, linear[1]*1000, linear[2]*1000, euler[0]*180/math.pi, euler[1]*180/math.pi, euler[2]*180/math.pi/60]
        #print("[MirbotEnv][generateGoal] goal: ", goal)
        return goal
    
    def getReward(self): 
        pose_diff = [g-c for g, c in zip(self.cart_goal, mirobot.current_pose)]
        distance = math.sqrt(sum([pow(x,2) for x in pose_diff[:3]]))
        distance_change = self.previous_distance - distance
        self.previous_distance = distance
        orientation_diff = sum(pose_diff[3:]) # sum of angle differcences
        orientation_change = self.previous_orientation_diff - orientation_diff
        self.previous_orientation_diff = orientation_diff
        force = (mirobot.average_force + mirobot.peak_force) * 10/2
        torque = (mirobot.average_torque + mirobot.peak_torque) * 200/2
        #print('[MirobotEnv] [getReward] Individual rewards  :', distance_change, orientation_change, force, torque)
        print('[MirobotEnv] [getReward] Overall rewards     :', distance_change + orientation_change/distance - force - torque)
        if distance > 10.0:
            return  distance_change + orientation_change/distance - force - torque
        return  distance_change + orientation_change - force - torque 

    def getNormalizedReward(self): 
        pose_diff = [g-c for g, c in zip(self.cart_goal, mirobot.current_pose)]
        
        distance = math.sqrt(sum([pow(x,2) for x in pose_diff[:3]]))
        distance_change = self.previous_distance - distance
        self.previous_distance = distance
        
        orientation_diff = sum(pose_diff[3:]) # sum of angle differcences
        orientation_change = self.previous_orientation_diff - orientation_diff
        self.previous_orientation_diff = orientation_diff
        
        force = mirobot.average_force + mirobot.peak_torque
        torque = mirobot.average_torque + mirobot.peak_force

        reward_array = np.array([distance_change, orientation_change/(distance/10), force, torque]) # for a high distance to the goal position, orientation doesn't matter
        #print('[MirobotEnv] [getNormalizedReward] Individual rewards:', reward_array)
        normalized_reward_array = preprocessing.normalize(reward_array)

        reward = np.sum(normalized_reward_array[:2]) - np.sum(normalized_reward_array[2:])
        #print('[MirobotEnv] [getNormalizedReward] Overall reward    :', reward)
        return reward

    def goalReached(self, obs):
        for current, goal in zip(obs, self.cart_goal):
            if abs(current - goal) > 0.5:
                return True
            return False