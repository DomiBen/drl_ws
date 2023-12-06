#!/usr/bin/env python3

import queue
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Duration
from std_msgs.msg import Header

sequence = 0
pub = rospy.Publisher('/joint_position_controller/command', JointTrajectory, queue_size = 100)

def callback(data):
    if data.position == [0, 0, 0, 0, 0, 0] : 
        print('[MIROBOT] Robot reached home position sucessfully!')
        print('[MIROBOT] %s interation(s)', sequence)
        quit()
    else:
        print('[MIROBOT] Publishing JointTrajectory message...')
        pubJointTrajectory()

def goHome():
    rospy.init_node('homeing', anonymous = True)
    rospy.Subscriber("/joint_states", JointState, callback)
    rate = rospy.Rate(50)
    rate.sleep()

def pubJointTrajectory():
    msg = JointTrajectory()
    msg.joint_names = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
    msg.header.seq = sequence + 1
    msg.header.stamp.secs = 0
    msg.header.stamp.nsecs = 0
    msg.header.frame_id = "/base_link"
    jtp1 = JointTrajectoryPoint()
    jtp1.positions = [0, 0, 0, 0, 0, 0]
    jtp1.velocities = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    jtp1.accelerations = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    jtp1.effort = [10, 10, 10, 10, 10, 10]
    jtp1.time_from_start.secs = 0
    jtp1.time_from_start.nsecs = 1
    msg.points.append(jtp1)
    pub.publish(msg)

if __name__ == '__main__':
    try: 
        goHome()
    except rospy.ROSInterruptException:
        pass 
