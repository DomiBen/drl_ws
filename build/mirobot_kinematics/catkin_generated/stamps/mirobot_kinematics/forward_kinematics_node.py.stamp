 #!/usr/bin/env python
import queue
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
kdl_kin = KDLKinematics(robot, "base_link", "link6")
pub = rospy.Publisher('endeffector_pose', Pose, queue_size = 100)
pose = Pose()

def publishPose():    
    #setting up ROS

    sub = rospy.Subscriber('joint_states', JointState, callback)
    rospy.init_node('MirobotPosePublisher', anonymous = True)
    rospy.spin()
    pass

def callback(data):
    #representing joint angles in vector
    # forward kinematics returns homogeneous transformation 4x4 numpy.mat
    kdl_pose = kdl_kin.forward(data.position)
    # extract euler and translation vectors from homogeneous transformation
    # linear position in kartesian coordinates
    point = Point()
    linear = translation_from_matrix(kdl_pose)
    point.x =  linear[0]
    point.y =  linear[1]
    point.z =  linear[2]
    pose.position = point
    # angular position in quaternions
    quat = Quaternion()
    angular = quaternion_from_matrix(kdl_pose) 
    quat.x = angular[0]
    quat.y = angular[1]
    quat.z = angular[2]
    quat.w = angular[3]
    pose.orientation = quat
    pub.publish(pose)
    pass

if __name__ == '__main__':
    try:
        publishPose()
    except rospy.ROSInterruptException:
        pass 