#!/usr/bin/env python3

#import rospy
import csv
import os 
import time 
import datetime


def write_to_csv(f_peak, f_avg, t_peak, t_avg):
    # Define the file path
    file_path = "/home/domi/drl_ws/src/sensor_logger/logfiles/sensor_data.csv"

    # Define the data to be written
    data = [f_peak, f_avg, t_peak, t_avg]

    # Open the file in append mode
    with open(file_path, mode='a', newline='') as file:
        # Create a CSV writer object
        writer = csv.writer(file)

        # Write the values as a row to the CSV file
        writer.writerow(data)
        

def write_dist(dist, dist_change, orient_change):
    # Define the file path
    file_path = "/home/domi/drl_ws/src/sensor_logger/logfiles/dist_data.csv"

    # Define the data to be written
    data = [dist, dist_change, orient_change]

    # Open the file in append mode
    with open(file_path, mode='a', newline='') as file:
        # Create a CSV writer object
        writer = csv.writer(file)

        # Write the values as a row to the CSV file
        writer.writerow(data)
        


def add_data_to_csv(distance, ft_reward, distance_change, orientation_change, distance_reward, orientation_reward, reward):
    # Define the file path
    file_path = "/home/domi/drl_ws/src/sensor_logger/logfiles/log_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"

    data = [distance, distance_change, orientation_change, ft_reward,  distance_reward, orientation_reward, reward]

    # Create the directory if it does not exist
    os.makedirs(os.path.dirname(file_path), exist_ok=True)

    # Write the data to the CSV file
    with open(file_path, mode='a', newline='') as file:
        # Create a CSV writer object
        writer = csv.writer(file)
        # Write the values as a row to the CSV file
        writer.writerow(data)