#!/usr/bin/env python
NAME = 'MirobotClient'

import rospy
import numpy as np
import math
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Vector3Stamped
from trajectory_planner.srv import *
from threading import Thread

class MirobotClient():
    def __init__(self):
        #self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.pose_sub = rospy.Subscriber('/endeffector_pose', Pose, self.pose_callback)
        self.current_pose = None
        self.record = False
        #For FT-Sensor usage
        self.force_sub = rospy.Subscriber("/force", Vector3Stamped, self.force_callback)
        self.force = np.array([0], dtype=np.float32)
        self.peak_force = 0
        self.average_force = 0
        self.torque_sub = rospy.Subscriber("/torque", Vector3Stamped, self.torque_callback)
        self.torque = np.array([0], dtype=np.float32)
        self.peak_torque = 0
        self.average_torque = 0
        #For IMU usage
        #self.lin_sub = rospy.Subscriber("/linacc", Vector3Stamped, self.force_callback)
        #self.ang_sub = rospy.Subscriber("/angvel", Vector3Stamped, self.torque_callback)
        #rospy setup 
        rospy.init_node(NAME)
        rospy.wait_for_service("/MirobotServer/SetJointRelativeCmd")
        rospy.wait_for_service("/MirobotServer/SetJointAbsoluteCmd")
        #print("[MirobotClient] /MirobotServer/SetJointRelativeCmd available")
    
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
        euler = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.current_pose = [data.position.x*1000, data.position.y*1000, data.position.z*1000, euler[0]*180/math.pi, euler[1]*180/math.pi, euler[2]*180/math.pi]

    def getObservation(self):
        return np.array(self.current_pose, dtype=np.float32)
    
    def executeAction(self, action):
        self.record = True
        #Service call
        #print("[MirobotClient] Calling Service")
        try:
            set_joint_service = rospy.ServiceProxy("/MirobotServer/SetJointAbsoluteCmd", SetJointCmd)
            req = SetJointCmdRequest()
            # type(action) > np.ndarray
            req.jointAngle_1 = action[0]
            req.jointAngle_2 = action[1]
            req.jointAngle_3 = action[2]
            req.jointAngle_4 = action[3]
            req.jointAngle_5 = action[4]
            req.jointAngle_6 = action[5]
            req.speed = round(action[6])
            response = set_joint_service(req)
            self.record = False
            #print("[MirobotClient] Executed action call sucessfully! ", response)
            #print("[MirobotClient] avg force: %s \t peak force: %s \n[MirobotClient] avg torque: %s \t peak torque: %s " % (self.average_force, self.peak_force, self.average_torque, self.peak_torque))
            return response
        except rospy.ServiceException as e:
            print("[MirobotClient] Service call failed: %s" %e)
        self.record = False

    def reset_ft_record(self):
        self.peak_force = 0
        self.average_force = 0
        self.peak_torque = 0
        self.average_torque = 0
