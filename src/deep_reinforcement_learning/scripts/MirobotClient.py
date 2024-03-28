#!/usr/bin/env python
# =============================================================================
# Created By  : Dominik Benchert
# 
# Last Update : April 2024
# License     : BSD-3
# =============================================================================
"""
This script is used to control the Mirobot robot arm via ROS services. It is used to connect it with the deep reinforcement learning environment.
"""
NAME = 'MirobotClient'

import rospy
import numpy as np
import math
import csv
import os
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Vector3Stamped
from trajectory_planner.srv import *
import numpy as np

class MirobotClient():
    def __init__(self):
        # setting up the subscribers
        self.pose_sub = rospy.Subscriber('/endeffector_pose', Pose, self.pose_callback)
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        self.current_point = None
        self.current_orientation = None
        self.current_joint_states = None
        # variables for enabling/disabling the force/torque recording
        self.record = False
        # variables for storing the force/torque values
        self.force = np.array([0], dtype=np.float32)
        self.peak_force = 0
        self.average_force = 0
        self.torque = np.array([0], dtype=np.float32)
        self.peak_torque = 0
        self.average_torque = 0
        ## For FT-sensor usage
        #self.force_sub = rospy.Subscriber("/force", Vector3Stamped, self.force_callback)
        #self.torque_sub = rospy.Subscriber("/torque", Vector3Stamped, self.torque_callback)
        #For IMU usage
        self.lin_sub = rospy.Subscriber("/linacc", Vector3Stamped, self.force_callback)
        self.ang_sub = rospy.Subscriber("/angvel", Vector3Stamped, self.torque_callback)
        #rospy setup 
        rospy.init_node(NAME)
        rospy.wait_for_service("/MirobotServer/SetJointRelativeCmd")
        rospy.wait_for_service("/MirobotServer/SetJointAbsoluteCmd")
        if not os.path.exists(self.logfile):
            os.makedirs(os.path.dirname(self.logfile), exist_ok=True)

    def force_callback(self, data): 
        if self.record:
            force_list = [data.vector.x, data.vector.y, data.vector.z]
            absolute_list = [abs(element) for element in force_list]
            # add values to force array
            self.force = np.append(self.force, [sum(absolute_list)])
            # calculate average force
            self.average_force = np.mean(self.force)
            # store highest recorded Force
            if max(absolute_list) > self.peak_force:
                self.peak_force = max(absolute_list)
            
    def torque_callback(self, data):
        if self.record:
            torque_list = [data.vector.x, data.vector.y, data.vector.z]
            absolute_list = [abs(element) for element in torque_list]
            # add values to torque array
            self.torque = np.append(self.torque, [sum(absolute_list)])
            self.average_torque = np.mean(self.torque)
            # store highest recorded torque
            if max(absolute_list) > self.peak_torque:
                self.peak_torque = max(absolute_list)

    def pose_callback(self, data):
        self.current_point = [data.position.x*1000, data.position.y*1000, data.position.z*1000]
        self.current_orientation = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        
        euler = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.current_pose = [data.position.x*1000, data.position.y*1000, data.position.z*1000, euler[0]*180/math.pi, euler[1]*180/math.pi, euler[2]*180/math.pi]
    
    def joint_callback(self, data):
        self.current_joint_states = [data.position[0]*180/math.pi, data.position[1]*180/math.pi, data.position[2]*180/math.pi, data.position[3]*180/math.pi, data.position[4]*180/math.pi, data.position[5]*180/math.pi]
    
    def executeAction(self, action):
        self.reset_ft_record()
        self.record = True #start recording force/torque values
        #Service call to move the robot joints
        try:
            move_joint_service = rospy.ServiceProxy("/MirobotServer/SetJointRelativeCmd", SetJointCmd)
            req = SetJointCmdRequest()
            # Map the values to the new range
            mapped_actions = np.interp(action, [0, 1, 2], [-1, 0, 1])
            # type(action) -> np.ndarray
            req.jointAngle_1 = mapped_actions[0]/2
            req.jointAngle_2 = mapped_actions[1]
            req.jointAngle_3 = mapped_actions[2]
            req.jointAngle_4 = mapped_actions[3]
            req.jointAngle_5 = mapped_actions[4]
            req.jointAngle_6 = mapped_actions[5]
            req.speed = 1000
            response = move_joint_service(req).result
            self.record = False # stop recording force/torque values
            return response
        except rospy.ServiceException as e:
            print("[MirobotClient] [executeAction] Service call failed: %s" %e)
        self.record = False
        return -1
        
    def moveToAbsolutePosition(self, pose):
        try:
            move_joint_service = rospy.ServiceProxy("/MirobotServer/SetJointAbsoluteCmd", SetJointCmd)
            req = SetJointCmdRequest()
            req.jointAngle_1 = pose[0]
            req.jointAngle_2 = pose[1]
            req.jointAngle_3 = pose[2]
            req.jointAngle_4 = pose[3]
            req.jointAngle_5 = pose[4]
            req.jointAngle_6 = pose[5]
            req.speed = 2000
            response = move_joint_service(req)
            return response
        except rospy.ServiceException as e:
            print("[MirobotClient] [executeAction] Service call failed: %s" %e)
        
    def reset_ft_record(self):
        self.peak_force = 0
        self.average_force = 0
        self.peak_torque = 0
        self.average_torque = 0
