#!/usr/bin/env python3
NAME = 'mirobot_mujoco_interface'

import math
import queue
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Duration, Header
from trajectory_planner.srv import *

sequence = 0

class ServiceServer(): 
    def __init__(self):
        self.pub = rospy.Publisher('/joint_position_controller/command', JointTrajectory, queue_size = 10)
        self.sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        #Setting up Services
        self.setJointSrv = rospy.Service('/MirobotServer/SetJointAbsolutCmd', SetJointCmd, self.set_joint_angles)
        self.setHomeSrv = rospy.Service('/MirobotServer/SetHomeCmd', SetHomeCmd, self.home_robot)
        self.moveJointSrv = rospy.Service('/MirobotServer/SetJointRelativeCmd', SetJointCmd, self.move_joints)
        self.getposeSrv = rospy.Service('/MirobotServer/GetPoseCmd', GetPoseCmd, self.get_pose)

        self.current_pose = None
        self.max_interation_until_timeout = 50

        rospy.init_node(NAME)
        self.rate = rospy.Rate(10)
        rospy.spin()
    
    def move_joints(self, req):
        msg = self.get_JointTrajectory(self.current_pose[0]+req.jointAngle_1, self.current_pose[1]+req.jointAngle_2, self.current_pose[2]+req.jointAngle_3, 
                                       self.current_pose[3]+req.jointAngle_4, self.current_pose[4]+req.jointAngle_5, self.current_pose[5]+req.jointAngle_6, req.speed)
        return self.execute_movement(msg)
        
    def home_robot(self, req):
        msg = self.get_JointTrajectory(0.0, 0.0, 0.0, 0.0, -1.57, 0.0, 10)
        return self.execute_movement(msg)

    def set_joint_angles(self, req):
        msg = self.get_JointTrajectory(req.jointAngle_1, req.jointAngle_2, req.jointAngle_3, req.jointAngle_4, req.jointAngle_5, req.jointAngle_6, req.speed)
        return self.execute_movement(msg)
    
    def execute_movement(self, msg):
        while self.wait_until_reached(msg):
            self.pub.publish(msg)
            self.rate.sleep()
        return 1

    def get_pose(self, req):

        return req

    def get_JointTrajectory(self, j1, j2, j3, j4, j5, j6, v):
        msg = JointTrajectory()
        msg.joint_names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
        msg.header.seq = sequence + 1
        msg.header.stamp.secs = 0
        msg.header.stamp.nsecs = 0
        msg.header.frame_id = "/base_link"
    
        jtp1 = JointTrajectoryPoint()
        jtp1.positions = [j1, j2, j3, j4, j5, j6]
        jtp1.velocities = [v, v, v, v, v, v]
        jtp1.accelerations = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        jtp1.effort = [1, 1, 1, 1, 1, 1]
        jtp1.time_from_start.secs = 0
        jtp1.time_from_start.nsecs = 1
        msg.points.append(jtp1)
        return msg

    def joint_state_callback(self, data):
        self.current_pose = data.position
        #print('Pose callback finished')

    def wait_until_reached(self, target):
        if self.current_pose is None:
            return True
        for current, goal in zip(self.current_pose, target.points[0].positions):
            if abs(current - goal) > 0.1:
                return True
        return False

    def joint_angles_legal(self, msg):
        for min, goal, max in zip(mirobot.min_angles_rad, msg.points[0].positions, mirobot.max_angles_rad):
            if min <= goal <= max:
                return True
            return False

class mirobot():
    def __init__(self):
        self.min_angles_deg = [-110, -35, -120, -180, -200, -360]
        self.min_angles_rad = self.min_angles_deg/2*math.pi
        self.max_angles_deg = [160, 70, 60, 180, 30, 360]
        self.max_angles_rad = self.max_angles_deg/2*math.pi

if __name__ == '__main__':
    try: 
        server = ServiceServer()
    except rospy.ROSInterruptException:
        pass 