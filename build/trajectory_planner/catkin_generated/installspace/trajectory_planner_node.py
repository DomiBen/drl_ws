#!/usr/bin/env python3

import queue
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Duration
from std_msgs.msg import Header

sequence = 0

def planner(): 
    pub = rospy.Publisher('/joint_position_controller/command', JointTrajectory, queue_size = 100)
    rospy.init_node('trajectory_planner', anonymous = True)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        traj_msgs = getJointTrajectory()
        pub.publish(traj_msgs)
        rate.sleep()

def getJointTrajectory():
    msg = JointTrajectory()
    msg.joint_names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
    msg.header.seq = sequence + 1
    msg.header.stamp.secs = 0
    msg.header.stamp.nsecs = 0
    msg.header.frame_id = "/base_link"
    jtp1 = JointTrajectoryPoint()
    jtp1.positions = [0.1, -0.25, -0.3, 0, 0, 0]
    jtp1.velocities = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    jtp1.accelerations = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    jtp1.effort = [10, 10, 10, 10, 10, 10]
    jtp1.time_from_start.secs = 0
    jtp1.time_from_start.nsecs = 1
 
    msg.points.append(jtp1)

    return msg

if __name__ == '__main__':
    try: 
        planner()
    except rospy.ROSInterruptException:
        pass 