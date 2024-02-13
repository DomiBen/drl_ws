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

# Poses for the robot to reach
START = [265, -30, 80, 0, -90, 0]
GOAL = [157, 120, 62, 0, 0, 0]

min_angles_deg = [-110, -35, -120, -180, -200, -360]
min_angles_rad = [i*math.pi/180 for i in min_angles_deg]
max_angles_deg = [160, 70, 60, 180, 30, 360]
max_angles_rad = [i*math.pi/180 for i in max_angles_deg]

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
        self.observation_space = spaces.Box(high=np.array([660, 473, 555, 660, 460, 360, 360, 360], dtype=np.float32),
                                            low=np.array([-660, -473, -555, -660, -460, -360, -360, -360], dtype=np.float32), dtype=np.float32)
        self.pose_counter = 0
        while(mirobot.current_pose == None): 
            sleep(1)
        self.stepcount = 0
        
        

    def step(self, action):
        #print('[MirobotEnv] [step] stepping with action: ', action)
        action_response = mirobot.executeAction(action)
        # creating reward
        self.reward = self.getScaledReward()
        #IMPORTATNT: distance and orientation_difference get updated in getReward() function!
        d_observation = np.array([self.previous_distance, self.previous_orientation_diff], dtype=np.float32)
        posediff_observation = np.array(self.pose_diff, dtype=np.float32)
        observation = np.concatenate((d_observation, posediff_observation))
        # check if terminated 
        if self.goalReached(mirobot.current_pose):
            self.terminated = True
            self.reward = self.reward + 500
        else: 
            self.terminated = False
        # check if Truncated
        if mirobot.current_pose[2] < 10 or action_response == -1 or np.all(action == 1):
            self.truncated = True
            self.reward = -500
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
        # generate new goal with random values for x, y, z, r, p, y
        self.goal = np.array(GOAL, dtype=np.float32)
        mirobot.moveToAbsolutePosition([-6.5, 52, -4, -8.7, -138.5, -5.7])
        # initialize previous distance and orientation difference for the reward function 
        self.pose_diff = [g-c for g, c in zip(self.goal, mirobot.current_pose)]
        self.previous_distance = math.sqrt(sum([pow(x,2) for x in self.pose_diff[:3]]))
        self.previous_orientation_diff = sum(self.pose_diff[3:])
        self.min_reached_distance = self.previous_distance
        # observation
        d_observation = np.array([self.previous_distance, self.previous_orientation_diff], dtype=np.float32)
        posediff_observation = np.array(self.pose_diff, dtype=np.float32)
        observation = np.concatenate((d_observation, posediff_observation))
        mirobot.reset_ft_record()
        info = {}
        self.stepcount = self.stepcount + 1
        return observation, info
    
    '''def generateGoal(self):
        if self.pose_counter % 2 == 0:
            goal = POSE_1
            mirobot.moveToAbsolutePosition([0, 52, -3, 0, -138.5, 0])
        else:
            goal = POSE_2
            mirobot.moveToAbsolutePosition([36.5, 49, 11, 40, -156, -77])
        self.pose_counter = self.pose_counter + 1
        print("[MirbotEnv][generateGoal] New goal: ", goal)
        return goal'''

    def goto(self, pose):
        #print('[MirobotEnv] [goto] going to pose: ', pose)
        goal = pose
        # then convert them to cartesian coordinates -> therefore we won't generate unreachable points
        joint_angles = kdl_kin.inverse(goal)
        mirobot.moveToAbsolutePosition(joint_angles)
        

    def getReward(self): 
        self.pose_diff = [g-c for g, c in zip(self.goal, mirobot.current_pose)]
        distance = math.sqrt(sum([pow(x,2) for x in self.pose_diff[:3]]))
        distance_change = self.previous_distance - distance
        self.previous_distance = distance
            
        orientation_diff = sum(self.pose_diff[3:])/3 # mean angle difference
        orientation_change = self.previous_orientation_diff - orientation_diff
        self.previous_orientation_diff = orientation_diff
        
        # force and torque multiplier calculated in /home/domi/drl_ws/src/sensor_logger/logfiles/sensor_data_calculation.ods
        #ft_reward = (mirobot.peak_force + mirobot.peak_torque*15)* 5 /2    #for ft usage
        ft_reward = (mirobot.peak_force + mirobot.peak_torque*64)* 3        #for imu usage
        #sensor_logger_node.write_to_csv(mirobot.average_force, mirobot.peak_force, mirobot.average_torque, mirobot.peak_torque)
        if distance_change > 0.05: 
            dist_reward = 50
        else:
            dist_reward = 0
        if orientation_change > 0.05:
            orientation_reward = 50
        else:
            orientation_reward = 0
        orientation_reward = orientation_reward * (20/max(10, distance)) # the further away from the goal, the less important is the orientation; maximum factor is 2 
        #sensor_logger_node.add_data_to_csv(distance, distance_change, orientation_change, dist_reward, orientation_reward, ft_reward, dist_reward + orientation_reward - ft_reward)
        reward = dist_reward + orientation_reward - ft_reward
        return reward
    
    def goalReached(self, obs):
        for current, goal in zip(obs[:3], self.goal[:3]):
            if abs(current - goal) > 15:                    # 15mm tolerance for the goalzone 
                return False
        print('[MirobotEnv] [goalReached] Goal reached!')
        return True
           
    def getScaledReward(self): 
        self.pose_diff = [g-c for g, c in zip(self.goal, mirobot.current_pose)]
        distance = math.sqrt(sum([pow(x,2) for x in self.pose_diff[:3]]))
        distance_change = self.min_reached_distance - distance
        self.previous_distance = distance
        self.min_reached_distance =  min(distance, self.min_reached_distance)
            
        orientation_diff = sum(self.pose_diff[3:])/3 # mean angle difference
        orientation_change = self.previous_orientation_diff - orientation_diff
        self.previous_orientation_diff = orientation_diff

        # force and torque multiplier calculated in /home/domi/drl_ws/src/sensor_logger/logfiles/sensor_data_calculation.ods
        #ft_reward = (mirobot.peak_force + mirobot.peak_torque*15)* 5 /2    #for ft usage
        ft_reward = (mirobot.peak_force + mirobot.peak_torque*64)* 3        #for imu usage
        #sensor_logger_node.write_to_csv(mirobot.average_force, mirobot.peak_force, mirobot.average_torque, mirobot.peak_torque)
        if distance_change > 0.01: 
            dist_reward = min(50, 50*500000/self.stepcount)
        else:
            dist_reward = 0
        if orientation_change > 0.01:
            orientation_reward = min(50, 50*500000/self.stepcount)
        else:
            orientation_reward = 0
        orientation_reward = orientation_reward * (20/max(10, distance)) # the further away from the goal, the less important is the orientation; maximum factor is 2 
        #sensor_logger_node.add_data_to_csv(distance, distance_change, orientation_change, dist_reward, orientation_reward, ft_reward, dist_reward + orientation_reward - ft_reward)
        reward = dist_reward + orientation_reward - ft_reward
        #print('[MirobotEnv] [getScaledReward] Reward: ', reward)
        return reward