#!/usr/bin/env python3

import rospy
import csv
import os 

def write_to_csv(f_peak, f_avg, t_peak, t_avg):
    # Define the file path
    file_path = "/home/domi/drl_ws/src/sensor_logger/logfiles/sensor_data.csv"
    if not os.path.exists(file_path):
        os.makedirs(file_path)

    # Define the data to be written
    data = [f_peak, f_avg, t_peak, t_avg]

    # Open the file in write mode
    with open(file_path, mode='w', newline='') as file:
        # Create a CSV writer object
        writer = csv.writer(file)

        # Write the data to the CSV file
        writer.writerows(data)
