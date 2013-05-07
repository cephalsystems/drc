#!/usr/bin/env python
#
# Read from two TF frames and attempt to create joint solutions that put the end
# effectors at those TF frames.
#

import roslib; roslib.load_manifest('kdl_hydra')
import rospy

from tf import TransformListener
from pykdl_utils import kdl_parser
from urdf_parser_py.urdf import URDF
import PyKDL as kdl

from razer_hydra.msg import Hydra
from sensor_msgs.msg import JointState


tf = None
robot_urdf = None
robot_kdl = None

left_ik = None
right_ik = None

left_chain = None
right_chain = None

left_q = None
right_q = None


def create_IK_solvers():
    global left_ik, right_ik, left_chain, right_chain

    # Define left and right arm chains
    left_chain = robot_kdl.getChain('utorso', 'l_hand')
    right_chain = robot_kdl.getChain('utorso', 'r_hand')

    # Initialize IK for left arm
    left_fk = kdl.ChainFkSolverPos_recursive(left_chain)
    left_ikv = kdl.ChainIkSolverVel_pinv(left_chain)
    left_ik = kdl.ChainIkSolverPos_NR(left_chain, left_fk, left_ikv, 100, 1e-6)

    # Initialize IK for right arm
    right_fk = kdl.ChainFkSolverPos_recursive(right_chain)
    right_ikv = kdl.ChainIkSolverVel_pinv(right_chain)
    right_ik = kdl.ChainIkSolverPos_NR(right_chain, right_fk, right_ikv, 100, 1e-6)


def atlas_callback(atlas_msg):
    global left_chain, right_chain
    global left_q, right_q

    # TODO: this can be optimized
    q = kdl.JntArray(left_chain.getNrOfJoints())
    for chain_idx in range(0, left_chain.getNrOfJoints()):

        # Get current joint name and use it to get joint index in state message
        name = left_chain.getSegment(chain_idx).getJoint().getName()
        tree_idx = atlas_msg.name.index(name)
        
        # Fill in corresponding joint index
        q[chain_idx] = atlas_msg.position[tree_idx]

    # Update global state 
    left_q = q

    q = kdl.JntArray(right_chain.getNrOfJoints())
    for chain_idx in range(0, right_chain.getNrOfJoints()):

        # Get current joint name and use it to get joint index in state message
        name = right_chain.getSegment(chain_idx).getJoint().getName()
        tree_idx = atlas_msg.name.index(name)
        
        # Fill in corresponding joint index
        q[chain_idx] = atlas_msg.position[tree_idx]

    # Update global state 
    right_q = q


def hydra_callback(hydra_msg):
    global tf, robot_urdf, robot_kdl
    global left_ik, right_ik
    global left_q, right_q

    # Verify that necessary frames and transforms exist
    if not tf.frameExists('/utorso'):
        rospy.logwarn('Unable to set hand goals: no torso frame')
        return

    if not tf.frameExists('/hydra_left'):
        rospy.logwarn('Unable to set hand goals: no hydra left frame')
        return

    if not tf.frameExists('/hydra_right'):
        rospy.logwarn('Unable to set hand goals: no hydra right frame')
        return

    # Set the current left hand target position
    try:
        # Compute the relative transform to the hand
        left_time = tf.getLatestCommonTime("/utorso", "/hydra_left")
        left_pos, left_quat = tf.lookupTransform("/utorso", "/hydra_left", left_time)
        left_target = kdl.Frame(kdl.Vector(left_pos[0], left_pos[1], left_pos[2]))

        # Compute IK for desired joint position
        left_desired = kdl.JntArray(left_q.rows())
        left_ret = left_ik.CartToJnt(left_q, left_target, left_desired);
        if (left_ret < 0):
            raise Exception('Left inverse kinematics failed with ' + str(left_ret))

        # TODO: send command to atlas!

    except Exception as e:
        rospy.logwarn('Unable to compute left hand target: %s', str(e))

    # Set the current right hand target position
    try:
        # Compute the relative transform to the hand
        right_time = tf.getLatestCommonTime("/utorso", "/hydra_right")
        right_pos, right_quat = tf.lookupTransform("/utorso", "/hydra_right", right_time)
        right_target = kdl.Frame(kdl.Vector(right_pos[0], right_pos[1], right_pos[2]))

        # Compute IK for desired joint position
        right_desired = kdl.JntArray(right_q.rows())
        right_ret = right_ik.CartToJnt(right_q, right_target, right_desired);
        if (right_ret < 0):
            raise Exception('Right inverse kinematics failed with ' + str(right_ret))

        # TODO: send command to atlas!

    except Exception as e:
        rospy.logwarn('Unable to compute left hand target: %s', str(e))


def main():
    global tf, robot_urdf, robot_kdl

    # Initialize the ROS node
    rospy.init_node('kdl_hydra')

    # Create a transform listener
    tf = TransformListener()
    
    # Retrieve raw robot parameters from rosmaster
    robot_string = rospy.get_param("robot_description")
    if not robot_string:
        raise Exception('Robot model not specified')

    # Load URDF model of robot description locally
    robot_urdf = URDF.parse_xml_string(robot_string)

    # Load URDF model of robot description into KDL
    robot_kdl = kdl_parser.kdl_tree_from_urdf_model(robot_urdf)

    # Initialize IK solvers
    create_IK_solvers()

    # Subscribe to hydra and atlas updates
    rospy.Subscriber("/hydra_calib", Hydra, hydra_callback)
    rospy.Subscriber("/atlas/joint_states", JointState, atlas_callback)

    # Start main event handling loop
    rospy.loginfo('Started kdl_hydra node...')
    rospy.spin()
    rospy.loginfo('Stopping kdl_hydra node...')


if __name__ == '__main__':
    main()
