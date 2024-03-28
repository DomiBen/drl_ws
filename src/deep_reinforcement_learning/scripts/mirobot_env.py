#!/usr/bin/env python
# =============================================================================
# Created By  : Dominik Benchert
# 
# Last Update : April 2024
# License     : BSD-3
# =============================================================================
"""
This script is used to create the custom environment for the Mirobot
"""
import gymnasium as gym
import numpy as np
import math
from gymnasium import spaces
from time import sleep
from pykdl_utils.kdl_kinematics import KDLKinematics
from urdf_parser_py.urdf import URDF
from MirobotClient import *
import random as r
from sklearn import preprocessing
from sensor_logger_node import ft_logger as log

# Start and Goal position for the path to be learned
START = [258, -30, 124, -0.0189, -0.7371, -0.1718, 0.6532]
GOAL = [140, 135, 60, 0.6553, -0.2651, 0.1444, 0.6923]

min_angles_deg = [-110, -35, -120, -180, -200, -360]
min_angles_rad = [i*math.pi/180 for i in min_angles_deg]
max_angles_deg = [160, 70, 60, 180, 30, 360]
max_angles_rad = [i*math.pi/180 for i in max_angles_deg]

# create MirobotClient object
mirobot = MirobotClient()
#load robot model from ros poarameter server and create KDLKinematiocs class
robot = URDF.from_parameter_server()
kdl_kin = KDLKinematics(robot, "base_link", "link61")

# Custom Environment that follows gym interface
class MirobotEnv(gym.Env):
    def __init__(self):
        super(MirobotEnv, self).__init__()
        # Define action and observation space -> must be gym space object 
        self.action_space = spaces.MultiDiscrete([3, 3, 3, 3, 3, 3]) # 6 joints with 3 possible actions each (0: -1°, 1: 0°, 2: +1°) and 4 possible speeds
        # Observation Space: distance to Goal, difference in orientation, and all Values within the workingspace of the robot -> Box with diffent sized Vectors
        # observation: distance to goal [1] | sum of orientation difference [1] | vector difference [3] | orientation difference as quaternion angles [4] | current joint states [6]
        self.observation_space = spaces.Box(high=np.array([660,                                 # max distance in mm
                                                           8,                                   # max orientation difference
                                                           555, 660, 460,                       # max vector difference in mm
                                                           2, 2, 2, 2,                          # max orientation difference in quaternion angles
                                                           160, 70, 60, 180, 30, 360],          # max joint states in degree
                                                          dtype=np.float32),
                                            low=np.array([0,                                    # min distance in mm
                                                          0,                                    # min orientation difference
                                                          -555, -660, -460,                     # min vector difference in mm
                                                          -2, -2, -2, -2,                       # min orientation difference in quaternion angles
                                                          -110, -35, -120, -180, -200, -360],   # min joint states in degree
                                                         dtype=np.float32), dtype=np.float32)
        
        while(mirobot.current_point == None) or (mirobot.current_orientation == None): 
            sleep(1)
        self.stepcount = 0

    def step(self, action):
        action_response = mirobot.executeAction(action)
        # creating reward
        self.reward = self.getScaledReward() #TODO: change to getScaledReward() for testing
        # observation: distance to goal | sum of orientation difference | vector difference | orientation difference as quaternion angles | current joint states
        d_observation = np.array([self.previous_distance, self.previous_orientation_diff], dtype=np.float32) #IMPORTATNT: distance and orientation_difference get updated in getReward() function!
        posediff_observation = np.concatenate([self.vector_diff, self.angle_diff], dtype=np.float32)
        observation = np.concatenate((d_observation, posediff_observation, mirobot.current_joint_states), dtype=np.float32)
        # check if terminated 
        if self.goalReached(mirobot.current_point, mirobot.current_orientation):
            self.terminated = True
            self.reward = self.reward + 1000
            print('[MirobotEnv] [step] Goal reached - Terminated!')
        else: 
            self.terminated = False
        # check if Truncated
        if mirobot.current_point[2] < 10 or action_response == -1 or np.all(action == 1):
            self.truncated = True
            self.reward = -1000
            print('[MirobotEnv] [step] Truncated!')
        else: 
            self.truncated = False
        info = {}             
        self.stepcount = self.stepcount + 1
        return observation, self.reward, self.terminated, self.truncated, info

    def reset(self, seed=None, options=None):
        # reset initiates Environment variables
        self.terminated = False
        self.truncated = False
        # generate goal and move robot to start position
        self.goal = np.array(GOAL, dtype=np.float32)
        mirobot.moveToAbsolutePosition([-6.5, 40, -5.5, -10.75, -130, 0]) # TODO: change to start position according to the start position of the path
        # initialize previous distance and orientation difference for the reward function 
        self.vector_diff = [g-c for g, c in zip(self.goal[:3], mirobot.current_point)]
        self.previous_distance = math.sqrt(sum([pow(x,2) for x in self.vector_diff]))
        self.angle_diff = [g-c for g, c in zip(self.goal[3:], mirobot.current_orientation)]
        self.previous_orientation_diff = sum([abs(a) for a in self.angle_diff])
        self.min_reached_distance = self.previous_distance
        self.min_orientation_diff = self.previous_orientation_diff
        # observation: distance to goal | sum of orientation difference | vector difference | orientation difference as quaternion angles | current joint states
        d_observation = np.array([self.previous_distance, self.previous_orientation_diff], dtype=np.float32)
        posediff_observation = np.concatenate([self.vector_diff, self.angle_diff], dtype=np.float32)
        observation = np.concatenate((d_observation, posediff_observation, mirobot.current_joint_states), dtype=np.float32)
        # reset the recorded values of the ft_record
        mirobot.reset_ft_record()
        info = {}
        self.stepcount = self.stepcount + 1
        return observation, info
        
    def goalReached(self, point, orientation):
        # distance of the current point to the goal
        for current, goal in zip(point, self.goal[:3]):
            # tolerance of 20mm in distance
            if abs(goal - current) > 20:
                return False
        # difference in orientation in quaternion angles
        for current, goal in zip(orientation, self.goal[3:]):
            # tolerance of 0.075 in quaternion angles
            if abs(goal - current) > 0.075:
                return False
        print('[MirobotEnv] [goalReached] Goal reached!')
        return True
           
    def getScaledReward(self): 
        # calculate the distance to the goal
        self.vector_diff = [g-c for g, c in zip(self.goal[:3], mirobot.current_point)]
        distance = math.sqrt(sum([pow(x,2) for x in self.vector_diff]))
        distance_change = self.min_reached_distance - distance
        self.previous_distance = distance
        #save the minimum distance reached so far
        self.min_reached_distance =  min(distance, self.min_reached_distance)
        
        # calculate the orientation difference to the goal
        self.angle_diff = [g-c for g, c in zip(self.goal[3:], mirobot.current_orientation)]
        orientation_diff = sum([abs(a) for a in self.angle_diff])
        orientation_change = self.min_orientation_diff - orientation_diff
        self.previous_orientation_diff = orientation_diff
        # save the minimum orientation difference reached so far
        self.min_orientation_diff = min(orientation_diff, self.min_orientation_diff)
        
        ### reward functions to choose from
        # force and torque multiplier calculated in src/sensor_logger/logfiles
        ft_reward = (mirobot.peak_force + mirobot.peak_torque*64)* 2  # best so far
        #ft_reward = ((mirobot.peak_force + mirobot.peak_torque*64)*3 + (mirobot.average_force + mirobot.average_torque*70)*2) * 0.3
        #ft_reward = (mirobot.average_force + mirobot.average_torque*70) * 1.08
        ### 
        # check if the distance to the goal has decreased
        if distance_change > 0: 
            dist_reward = 50
        else:
            dist_reward = -20
        # check if the orientation difference has decreased
        if orientation_change > 0:
            orientation_reward = 50
        else:
            orientation_reward = -20
        orientation_reward = orientation_reward * (75/max(50, distance)) # Scaling: the further away from the goal, the less important is the orientation; maximum factor is 2 
        # calculate the final reward, scaling the reward with the stepcount did not improve the learning
        reward = (dist_reward + orientation_reward)*min(1, 1*500000/self.stepcount) - ft_reward
        return reward
    
    def getReward(self):
        # calculate the distance to the goal
        self.vector_diff = [g-c for g, c in zip(self.goal[:3], mirobot.current_point)]
        distance = math.sqrt(sum([pow(x,2) for x in self.vector_diff]))
        distance_change = self.min_reached_distance - distance
        self.previous_distance = distance
        #save the minimum distance reached so far
        self.min_reached_distance =  min(distance, self.min_reached_distance)
        
        # calculate the orientation difference to the goal
        self.angle_diff = [g-c for g, c in zip(self.goal[3:], mirobot.current_orientation)]
        orientation_diff = sum([abs(a) for a in self.angle_diff])
        orientation_change = self.min_orientation_diff - orientation_diff
        self.previous_orientation_diff = orientation_diff
        # save the minimum orientation difference reached so far
        self.min_orientation_diff = min(orientation_diff, self.min_orientation_diff)
        
        ### reward functions to choose from
        # force and torque multiplier calculated in src/sensor_logger/logfiles
        ft_reward = (mirobot.peak_force + mirobot.peak_torque*64)* 2  # best so far
        #ft_reward = ((mirobot.peak_force + mirobot.peak_torque*64)*3 + (mirobot.average_force + mirobot.average_torque*70)*2) * 0.3
        #ft_reward = (mirobot.average_force + mirobot.average_torque*70) * 1.08
        ### 
        # check if the distance to the goal has decreased
        if distance_change > 0: 
            dist_reward = 50
        else:
            dist_reward = -20
        # check if the orientation difference has decreased
        if orientation_change > 0:
            orientation_reward = 50
        else:
            orientation_reward = -20
        orientation_reward = orientation_reward * (75/max(50, distance)) # Scaling: the further away from the goal, the less important is the orientation; maximum factor is 2 
        # calculate the final reward, scaling the reward with the stepcount did not improve the learning
        reward = dist_reward + orientation_reward - ft_reward
        return reward