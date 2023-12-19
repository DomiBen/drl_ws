#!/usr/bin/env python
NAME = 'MirobotServer'

import math
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
#from std_msgs.msg import Duration, Header
from trajectory_planner.srv import *
from tf.transformations import euler_from_quaternion

sequence = 0

class ServiceServer(): 
    def __init__(self):
        self.pub = rospy.Publisher('/joint_position_controller/command', JointTrajectory, queue_size = 10)
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.pose_sub = rospy.Subscriber('/endeffector_pose', Pose, self.pose_callback)
        #Setting up Services
        self.setJointSrv = rospy.Service('/MirobotServer/SetJointAbsoluteCmd', SetJointCmd, self.set_joint_angles)
        self.setHomeSrv = rospy.Service('/MirobotServer/SetHomeCmd', SetHomeCmd, self.home_robot)
        self.moveJointSrv = rospy.Service('/MirobotServer/SetJointRelativeCmd', SetJointCmd, self.move_joints)
        self.getposeSrv = rospy.Service('/MirobotServer/GetPoseCmd', GetPoseCmd, self.get_pose)

        self.max_interation_until_timeout = 50

        rospy.init_node(NAME)
        self.rate = rospy.Rate(10)
        rospy.spin()
    
    def move_joints(self, req):
        # request in degree | msgs in rad
        msg = self.get_JointTrajectory(mirobot.current_joint_states_rad[0]+req.jointAngle_1*math.pi/180, mirobot.current_joint_states_rad[1]+req.jointAngle_2*math.pi/180, 
                                       mirobot.current_joint_states_rad[2]+req.jointAngle_3*math.pi/180, mirobot.current_joint_states_rad[3]+req.jointAngle_4*math.pi/180, 
                                       mirobot.current_joint_states_rad[4]+req.jointAngle_5*math.pi/180, mirobot.current_joint_states_rad[5]+req.jointAngle_6*math.pi/180, 
                                       req.speed*math.pi/180)
        return self.execute_movement(msg)
        
    def home_robot(self, req):
        msg = self.get_JointTrajectory(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10)
        return self.execute_movement(msg)

    def set_joint_angles(self, req):
        msg = self.get_JointTrajectory(req.jointAngle_1*math.pi/180, req.jointAngle_2*math.pi/180, req.jointAngle_3*math.pi/180, 
                                       req.jointAngle_4*math.pi/180, req.jointAngle_5*math.pi/180, req.jointAngle_6*math.pi/180, 
                                       req.speed*math.pi/180)
        return self.execute_movement(msg)

    def get_pose(self, req):
        if mirobot.current_joint_states_deg and mirobot.current_pose is not None:
            return 1, -1, mirobot.current_pose[0], mirobot.current_pose[1], mirobot.current_pose[2], mirobot.current_pose[3], mirobot.current_pose[4], mirobot.current_pose[5], mirobot.current_joint_states_deg[0], mirobot.current_joint_states_deg[1], mirobot.current_joint_states_deg[2], mirobot.current_joint_states_deg[3], mirobot.current_joint_states_deg[4], mirobot.current_joint_states_deg[5], 0
        return 0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0
    
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
        mirobot.current_joint_states_rad = data.position
        mirobot.current_joint_states_deg = [i*180/math.pi for i in mirobot.current_joint_states_rad ]

    def pose_callback(self, data):
        euler = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        mirobot.current_pose = [data.position.x, data.position.y, data.position.z, euler[0], euler[1], euler[2]]

    def execute_movement(self, msg):
        iteration = 0
        if self.joint_angles_legal(msg):
            while self.wait_until_reached(msg):
                if iteration < self.max_interation_until_timeout:
                    self.pub.publish(msg)
                    self.rate.sleep()
                    iteration = iteration+1
                else:
                    print("[MirobotServer] Timeout while executing movement!")
                    return -1
            return 1
        return -1

    def wait_until_reached(self, target):
        if mirobot.current_joint_states_rad is None:
            return True
        for current, goal in zip(mirobot.current_joint_states_rad, target.points[0].positions):
            if abs(current - goal) > 0.2:
                return True
        print("[MirobotServer] execution done:", mirobot.current_pose, "!")
        return False

    def joint_angles_legal(self, msg):
        for i in range(len(msg.points[0].positions)):
            if msg.points[0].positions[i] < mirobot.min_angles_rad[i] or msg.points[0].positions[i] > mirobot.max_angles_rad[i]:
                print("[MirobotServer] Joint angles not allowed: Terminating movement!")
                return False
        #print("[MirobotServer] Joint angles allowed: Proceed to execution!")
        return True

class Mirobot():
    def __init__(self):
        self.min_angles_deg = [-110, -35, -120, -180, -200, -360]
        self.min_angles_rad = [i*math.pi/180 for i in self.min_angles_deg]

        self.max_angles_deg = [160, 70, 60, 180, 30, 360]
        self.max_angles_rad = [i*math.pi/180 for i in self.max_angles_deg]

        self.current_joint_states_rad = None
        self.current_joint_states_deg = None
        self.current_pose = None

if __name__ == '__main__':
    try: 
        mirobot = Mirobot()
        server = ServiceServer()
    except rospy.ROSInterruptException:
        pass 