 #!/usr/bin/env python

import rospy
import math
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from tf.transformations import quaternion_from_matrix, euler_from_matrix, quaternion_from_euler
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
    point.x =  linear[0] + 0.0035
    point.y =  linear[1] - 0.011295241815272498
    point.z =  linear[2]
    pose.position = point
    # angular
    # angular position in quaternions
    quat = Quaternion()
    
    '''angular_e = euler_from_matrix(kdl_pose)
    angular_e = [angular_e[0] - math.pi/2 , angular_e[1], angular_e[2] + math.pi / 2]
    angular_q = quaternion_from_euler(angular_e[0], angular_e[1], angular_e[2])'''
    
    angular_q = quaternion_from_matrix(kdl_pose)
    
    quat.x = angular_q[0]
    quat.y = angular_q[1]
    quat.z = angular_q[2]
    quat.w = angular_q[3]
    pose.orientation = quat
    pub.publish(pose)
    pass

if __name__ == '__main__':
    try:
        publishPose()
    except rospy.ROSInterruptException:
        pass 