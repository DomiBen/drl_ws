#!/usr/bin/env python
# =============================================================================
# Created By  : Dominik Benchert
# 
# Last Update : April 2024
# License     : BSD-3
# =============================================================================
"""
This node is used to publish the cartesian pose of the Mirobot. The node subscribes to the joint_states topic and uses the forward kinematics to calculate the cartesian pose.
"""

import rospy
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from tf.transformations import quaternion_from_matrix
from tf.transformations import translation_from_matrix
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState

#load robopt model from ros poarameter server
robot = URDF.from_parameter_server()
# creating KDLKinematiocs class
kdl_kin = KDLKinematics(robot, "base_link", "link61")
pub = rospy.Publisher('endeffector_pose', Pose, queue_size = 100)
pose = Pose()

def publishPose():    
    #setting up ROS node and subscriber
    rospy.init_node('MirobotPosePublisher', anonymous = True)
    sub = rospy.Subscriber('joint_states', JointState, callback)
    rospy.spin()
    pass

def callback(data):
    #representing joint angles in vector
    # forward kinematics returns homogeneous transformation 4x4 numpy.mat
    kdl_pose = kdl_kin.forward(data.position, end_link="link6", base_link="base_link")
    # extract euler and translation vectors from homogeneous transformation
    # linear position in kartesian coordinates
    point = Point()
    linear = translation_from_matrix(kdl_pose)
    # linear position in kartesian coordinates, offset to the endeffector
    point.x =  linear[0] + 0.0035
    point.y =  linear[1] - 0.011295241815272498
    point.z =  linear[2]
    pose.position = point
    # angular
    # angular position in quaternions
    quat = Quaternion()
    
    angular_q = quaternion_from_matrix(kdl_pose)
    
    quat.x = angular_q[0]
    quat.y = angular_q[1]
    quat.z = angular_q[2]
    quat.w = angular_q[3]
    pose.orientation = quat
    pub.publish(pose)
    pass

if _s_name__ == '__main__':
    try:
        publishPose()
    except rospy.ROSInterruptException:
        pass 