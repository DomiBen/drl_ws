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
import sensor_logger_node

min_angles_deg = [-110, -35, -120, -180, -200, -360]
min_angles_rad = [i*math.pi/180 for i in min_angles_deg]
max_angles_deg = [160, 70, 60, 180, 30, 360]
max_angles_rad = [i*math.pi/180 for i in max_angles_deg]

mirobot = MirobotClient()
#load robot model from ros poarameter server and create KDLKinematiocs class
robot = URDF.from_parameter_server()
kdl_kin = KDLKinematics(robot, "base_link", "link6")

# Custom Environment that follows gym interface
class MirobotEnv(gym.Env):
    def __init__(self):
        super(MirobotEnv, self).__init__()
        # Define action and observation space -> must be gym space object 
        self.action_space = spaces.MultiDiscrete([3, 3, 3, 3, 3, 3, 10]) # 6 joints with 3 possible actions each (0: -0.0025°, 1: 0°, 2: +0.0025°) and 10 possible speeds (200, 400, 600,..., 2000)
        # Observation Space: distance to Goal, difference in orientation, and all Values within the workingspace of the robot -> Box with diffent sized Vectors
        self.observation_space = spaces.Box(high=np.array([660, 473, 555, 660, 460, 360, 360, 360], dtype=np.float32),
                                            low=np.array([-660, -473, -555, -660, -460, -360, -360, -360], dtype=np.float32), dtype=np.float32)
        while(mirobot.current_pose == None): 
            sleep(1)

    def step(self, action):
        #print('[MirobotEnv] [step] stepping with action: ', action)
        action_response = mirobot.executeAction(action)
        # creating reward
        self.reward = self.getReward()
        #IMPORTATNT: distance and orientation_difference get updated in getReward() function!
        d_observation = np.array([self.previous_distance, self.previous_orientation_diff], dtype=np.float32)
        posediff_observation = np.array(self.pose_diff, dtype=np.float32)
        observation = np.concatenate((d_observation, posediff_observation))
        # check if terminated 
        if self.goalReached(mirobot.current_pose):
            self.terminated = True
            self.reward = self.reward + 1000
        else: 
            self.terminated = False
        # check if Truncated
        if mirobot.current_pose[2] < 1 or action_response == -1:
            self.truncated = True
            self.reward = self.reward - 1000
            print('[MirobotEnv] [step] Truncated!')
        else: 
            self.truncated = False

        info = {}
        self.previous_action = action
        
        return observation, self.reward, self.terminated, self.truncated, info

    def reset(self, seed=None, options=None):
        # reset initiates Environment variables
        self.terminated = False
        self.truncated = False
        # generate new goal with random values for x, y, z, r, p, y
        self.goal = np.array(self.generateGoal(), dtype=np.float32)
        # initialize previous distance and orientation difference for the reward function 
        self.pose_diff = [g-c for g, c in zip(self.goal, mirobot.current_pose)]
        self.previous_distance = math.sqrt(sum([pow(x,2) for x in self.pose_diff[:3]]))
        self.previous_orientation_diff = sum(self.pose_diff[3:])# euclidean distance
        # observation
        d_observation = np.array([self.previous_distance, self.previous_orientation_diff], dtype=np.float32)
        posediff_observation = np.array(self.pose_diff, dtype=np.float32)
        p_observation = mirobot.getPoseObservation() # returns np.array([cart_x, cart_y, cart_z, euler_r, euler_p, euler_y])
        observation = np.concatenate((d_observation, posediff_observation, p_observation))
        
        mirobot.reset_ft_record()
        info = {}
        return observation, info
    
   #def close(): 
    
    def generateGoal(self):
        goal = [0, 0, 0, 0, 0, 0]
        while goal[2] < 10:
            #first pick random values for the joint angles, between theire minumum and maximum constraints
            rand_joint_states = [r.uniform(min_angles_rad[0], max_angles_rad[0]), r.uniform(min_angles_rad[1], max_angles_rad[1]), r.uniform(min_angles_rad[2], max_angles_rad[2]), 
                                r.uniform(min_angles_rad[3], max_angles_rad[3]), r.uniform(min_angles_rad[4], max_angles_rad[4]), r.uniform(min_angles_rad[5], max_angles_rad[5])]
            # then convert them to cartesian coordinates -> therefore we won't generate unreachable points
            pose = kdl_kin.forward(rand_joint_states)
            linear = translation_from_matrix(pose)
            euler = euler_from_matrix(pose)
            goal = [linear[0]*1000, linear[1]*1000, linear[2]*1000, euler[0]*180/math.pi, euler[1]*180/math.pi, euler[2]*180/math.pi]
        
        print("[MirbotEnv][generateGoal] New goal: ", goal)
        return goal
    

    def getReward(self): 
        self.pose_diff = [g-c for g, c in zip(self.goal, mirobot.current_pose)]
        distance = math.sqrt(sum([pow(x,2) for x in self.pose_diff[:3]]))
        distance_change = self.previous_distance - distance
        self.previous_distance = distance
            
        orientation_diff = sum(self.pose_diff[3:])/3 # mean angle difference
        orientation_change = self.previous_orientation_diff - orientation_diff
        self.previous_orientation_diff = orientation_diff

        # force and torque multiplier calculated in /home/domi/drl_ws/src/sensor_logger/logfiles/sensor_data_calculation.ods
        ft_reward = (mirobot.average_force*3.33 + mirobot.peak_force + mirobot.average_torque*67 + mirobot.peak_torque*15)
        if distance_change > 1: 
            dist_reward = 50
        else:
            dist_reward = 0
        
        if orientation_diff > 1:
            orientation_reward = 50
        else:
            orientation_reward = 0
        orientation_reward = orientation_reward * (20/max(10, distance)) # the further away from the goal, the less important is the orientation; maximum factor is 2 

        sensor_logger_node.add_data_to_csv(distance, distance_change, orientation_change, dist_reward, orientation_reward, ft_reward, dist_reward + orientation_reward - ft_reward)

        reward = dist_reward + orientation_reward - ft_reward
        return reward
    
    '''def getReward(self): 
        self.pose_diff = [g-c for g, c in zip(self.goal, mirobot.current_pose)]
        distance = math.sqrt(sum([pow(x,2) for x in self.pose_diff[:3]]))
        distance_change = self.previous_distance - distance
        self.previous_distance = distance
        
        orientation_diff = sum(self.pose_diff[3:]) # sum of angle differcences
        orientation_change = self.previous_orientation_diff - orientation_diff
        self.previous_orientation_diff = orientation_diff
        # force and torque multiplier calculated in /home/domi/drl_ws/src/sensor_logger/logfiles/sensor_data_calculation.ods
        force = mirobot.average_force*3.33 + mirobot.peak_force
        torque = mirobot.average_torque*67 + mirobot.peak_torque*15
        
        sensor_logger_node.write_dist(distance, distance_change, orientation_change)
        
        if distance > 10.0:
            return  distance_change + orientation_change/distance - force - torque
        return  distance_change + orientation_change - force - torque

    def getNormalizedReward(self): 
        self.pose_diff = [g-c for g, c in zip(self.goal, mirobot.current_pose)]
        
        distance = math.sqrt(sum([pow(x,2) for x in self.pose_diff[:3]]))
        distance_change = self.previous_distance - distance
        self.previous_distance = distance
        
        orientation_diff = sum(self.pose_diff[3:]) # sum of angle differcences
        orientation_change = self.previous_orientation_diff - orientation_diff
        self.previous_orientation_diff = orientation_diff
        
        force = mirobot.average_force + mirobot.peak_torque
        torque = mirobot.average_torque + mirobot.peak_force

        reward_array = np.array([distance_change, orientation_change/(distance/10), force, torque]) # for a high distance to the goal position, orientation doesn't matter
        print('[MirobotEnv] [getNormalizedReward] Individual rewards:', reward_array)
        normalized_reward_array = preprocessing.normalize(reward_array)

        reward = sum(normalized_reward_array[:2]) - sum(normalized_reward_array[2:])
        print('[MirobotEnv] [getNormalizedReward] Overall reward    :', reward)
        return reward
'''
    
    def goalReached(self, obs):
        for current, goal in zip(obs, self.goal):
            if abs(current - goal) < 0.3:
                print('[MirobotEnv] [goalReached] Goal reached!')
                return True
            return False