#!/usr/bin/env python
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
    file_path = "/home/domi/drl_ws/src/sensor_logger/logfiles/IMU_log.csv"
    # Define the data to be written
    data = [datetime.datetime.now().strftime("%H:%M:%S"), f_peak, f_avg, t_peak, t_avg]
    # Open the file in append mode
    with open(file_path, mode='a', newline='') as file:
        # Create a CSV writer object
        writer = csv.writer(file)
        # Write the values as a row to the CSV file
        writer.writerow(data)

def log_action(action):
    file_path = "/home/dominik/drl_ws/src/sensor_logger/logfiles/action_log_" + STARTTIME + ".csv"
    # Define the data to be written
    data = [action[0], action[1], action[2], action[3], action[4], action[5]]
    # Open the file in append mode
    with open(file_path, mode='a', newline='') as file:
        # Create a CSV writer object
        writer = csv.writer(file)
        # Write the values as a row to the CSV file
        writer.writerow(data)


class Logger():
    def __init__(self, label):
        self.record = False
        self.starttime = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        #For Force/Torque Sensor readings
        self.force_sub = rospy.Subscriber("/force", Vector3Stamped, self.force_logger_callback)
        self.force = 0
        self.max_force = 0
        self.torque_sub = rospy.Subscriber("/torque", Vector3Stamped, self.torque_logger_callback)
        self.torque = 0
        self.max_torque = 0
        #For IMU sensor readings
        self.lin_sub = rospy.Subscriber("/linacc", Vector3Stamped, self.linacc_logger_callback)
        self.linacc = 0
        self.max_linacc = 0
        self.ang_sub = rospy.Subscriber("/angvel", Vector3Stamped, self.angvel_logger_callback)
        self.angvel = 0
        self.max_angvel = 0
        # file paths
        self.imu_file_path = "/home/domi/drl_ws/src/sensor_logger/logfiles/IMU_recording_" + label + "_" + self.starttime + ".csv"
        self.ft_file_path = "/home/domi/drl_ws/src/sensor_logger/logfiles/FT_recording_" + label + "_" + self.starttime + ".csv"
        #rospy setup 
        rospy.init_node(NAME)
        print("ftLogger node started")
        rate = rospy.Rate(100)
        while(not rospy.is_shutdown()):
            if self.record:
                self.write_data()
            rate.sleep()
        

    def force_logger_callback(self, data):
        self.force = (data.vector.x + data.vector.y + data.vector.z)/3
        self.max_force = max(data.vector.x, data.vector.y, data.vector.z)
    
    def torque_logger_callback(self, data):
        self.torque = (data.vector.x + data.vector.y + data.vector.z)/3
        self.max_torque = max(data.vector.x, data.vector.y, data.vector.z)
        
    def linacc_logger_callback(self, data):
        self.linacc = (data.vector.x + data.vector.y + data.vector.z)/3
        self.max_linacc = max(data.vector.x, data.vector.y, data.vector.z)
        
    def angvel_logger_callback(self, data):
        self.angvel = (data.vector.x + data.vector.y + data.vector.z)/3
        self.max_angvel = max(data.vector.x, data.vector.y, data.vector.z)
    
    def write_data(self):
        # Define the data to be written
        imu_data = [self.linacc, self.max_linacc, self.angvel, self.max_angvel]
        ft_data = [self.force, self.max_force, self.torque, self.max_torque]
        # Open the file in append mode
        with open(self.imu_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(imu_data)
        with open(self.ft_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(ft_data)

if __name__ == '__main__':
    try: 
        Logger()
    except rospy.ROSInterruptException: pass