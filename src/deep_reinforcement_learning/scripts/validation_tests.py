from mirobot_env import *
from MirobotClient import *
from sb3_contrib import TRPO
import torch as th
import os 
import csv
import datetime
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from sensor_logger_node import Logger

# Setting up parameters
csv_file_path = "/home/domi/drl_ws/src/sensor_logger/logfiles/action_log_linear_reward.csv"
MODELNAME = "TRPO_linear_reward"

mirobot_client = MirobotClient()
mirobot_client.moveToAbsolutePosition([-6.5, 40, -5.5, -10.75, -130, 0])

sensor_logger = Logger("TRPO_linear_reward")

# Open the CSV file in read mode
with open(csv_file_path, mode='r') as file:
    # Create a CSV reader object
    reader = csv.reader(file)
    sensor_logger.record = True
    # Loop over all lines in the CSV file
    for row in reader:
        # Create a list from the current row
        row_list = list(row)
        action = np.array(row_list, dtype=np.int32)
        mirobot_client.executeAction(action)
    sensor_logger.record = False
