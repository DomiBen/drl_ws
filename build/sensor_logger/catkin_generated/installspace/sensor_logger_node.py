#!/usr/bin/env python3
NAME = 'sensor_logger_node'
#import rospy
import csv
import os 
import time 
import datetime
import rospy
from geometry_msgs.msg import Pose, Vector3Stamped

STARTTIME = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

def ft_logger(force, torque, linacc, angvel):
    # Define the file path
    file_path = "/home/domi/drl_ws/src/sensor_logger/logfiles/sensor_data_" + STARTTIME + ".csv"
    # Define the data to be written
    data = [datetime.datetime.now().strftime("%H:%M:%S"), force, torque, linacc, angvel]
    # Open the file in append mode
    with open(file_path, mode='a', newline='') as file:
        # Create a CSV writer object
        writer = csv.writer(file)
        # Write the values as a row to the CSV file
        writer.writerow(data)
        
        
def log_imu_data(f_peak, f_avg, t_peak, t_avg):
    # Define the file path
    file_path = "/home/domi/drl_ws/src/sensor_logger/logfiles/IMU_log_" + STARTTIME + ".csv"
    # Define the data to be written
    data = [datetime.datetime.now().strftime("%H:%M:%S"), f_peak, f_avg, t_peak, t_avg]
    # Open the file in append mode
    with open(file_path, mode='a', newline='') as file:
        # Create a CSV writer object
        writer = csv.writer(file)
        # Write the values as a row to the CSV file
        writer.writerow(data)

def log_action(action):
    file_path = "/home/domi/drl_ws/src/sensor_logger/logfiles/action_log_" + STARTTIME + ".csv"
    # Define the data to be written
    data = [action[0], action[1], action[2], action[3], action[4], action[5]]
    # Open the file in append mode
    with open(file_path, mode='a', newline='') as file:
        # Create a CSV writer object
        writer = csv.writer(file)
        # Write the values as a row to the CSV file
        writer.writerow(data)
    

def add_data_to_csv(distance, ft_reward, distance_change, orientation_change, distance_reward, orientation_reward, reward):
    # Define the file path
    file_path = "/home/domi/drl_ws/src/sensor_logger/logfiles/log_" + STARTTIME + ".csv"
    data = [distance, distance_change, orientation_change, ft_reward,  distance_reward, orientation_reward, reward]
    # Create the directory if it does not exist
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    # Write the data to the CSV file
    with open(file_path, mode='a', newline='') as file:
        # Create a CSV writer object
        writer = csv.writer(file)
        # Write the values as a row to the CSV file
        writer.writerow(data)

class ftLogger():
    def __init__(self):
        self.force_sub = rospy.Subscriber("/force", Vector3Stamped, self.force_logger_callback)
        self.force = 0
        self.torque_sub = rospy.Subscriber("/torque", Vector3Stamped, self.torque_logger_callback)
        self.torque = 0
        #For IMU usage
        self.lin_sub = rospy.Subscriber("/linacc", Vector3Stamped, self.linacc_logger_callback)
        self.linacc = 0
        self.ang_sub = rospy.Subscriber("/angvel", Vector3Stamped, self.angvel_logger_callback)
        self.angvel = 0
        #rospy setup 
        rospy.init_node(NAME)
        print("ftLogger node started")
        rate = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            self.write_data()
            rate.sleep()
        

    def force_logger_callback(self, data):
        self.force = data.vector.x + data.vector.y + data.vector.z
    
    def torque_logger_callback(self, data):
        self.torque = data.vector.x + data.vector.y + data.vector.z
        
    def linacc_logger_callback(self, data):
        self.linacc = data.vector.x + data.vector.y + data.vector.z
        
    def angvel_logger_callback(self, data):
        self.angvel = data.vector.x + data.vector.y + data.vector.z
    
    def write_data(self):
        ft_logger(self.force, self.torque, self.linacc, self.angvel)

if __name__ == '__main__':
    try: 
        STARTTIME = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        ftLogger()
    except rospy.ROSInterruptException: pass