#!/usr/bin/env python
# =============================================================================
# Created By  : Dominik Benchert
# 
# Last Update : April 2024
# License     : BSD-3
# =============================================================================
"""
This script is used to check the performance of the trained model in the real World and record the sensor data.
"""

from mirobot_env import *
from MirobotClient import *
import csv
from sensor_logger_node import Logger

# Setting the path to the Action log file
csv_file_path = "/home/domi/drl_ws/src/sensor_logger/logfiles/action_log_linear_reward.csv"
# Setting the model name
MODELNAME = "TRPO_linear_reward"
# creat MirobotClient object
mirobot_client = MirobotClient()
mirobot_client.moveToAbsolutePosition([-6.5, 40, -5.5, -10.75, -130, 0])
sensor_logger = Logger(MODELNAME)

# Open the CSV file in read mode
with open(csv_file_path, mode='r') as file:
    # Create a CSV reader object
    reader = csv.reader(file)
    sensor_logger.record = True
    # Loop over all lines in the CSV file and read them to execute the actions
    for row in reader:
        # Create a list from the current row 
        row_list = list(row)
        # execute the action
        action = np.array(row_list, dtype=np.int32)
        mirobot_client.executeAction(action)
    sensor_logger.record = False
