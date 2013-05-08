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
from numpy import zeros

from razer_hydra.msg import Hydra
from sensor_msgs.msg import JointState
from atlas_msgs.msg import AtlasCommand

import threading

atlasJointNames = [
  'back_lbz', 'back_mby', 'back_ubx', 'neck_ay',
  'l_leg_uhz', 'l_leg_mhx', 'l_leg_lhy', 'l_leg_kny', 'l_leg_uay', 'l_leg_lax',
  'r_leg_uhz', 'r_leg_mhx', 'r_leg_lhy', 'r_leg_kny', 'r_leg_uay', 'r_leg_lax',
  'l_arm_usy', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx',
  'r_arm_usy', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx']

kp = {}
ki = {}
kd = {}
i_clamp = {}

tf = None
robot_urdf = None
robot_kdl = None

left_ik = None
right_ik = None

left_fk = None
right_fk = None

left_ikv = None
right_ikv = None

left_chain = None
right_chain = None

left_q = None
right_q = None

left_desired = None
right_desired = None

lock = threading.Lock()
pub = None

def load_gains(atlas_msg):
    global kp, ki, kd

    # Only load gains once
    if kp or ki or kd:
        return
    
    # Load each gain into dictionary
    for i in xrange(len(atlas_msg.name)):
        name = atlas_msg.name[i]
        kp[name] = rospy.get_param('atlas_controller/gains/' + name + '/p')
        ki[name] = rospy.get_param('atlas_controller/gains/' + name + '/i')
        kd[name] = rospy.get_param('atlas_controller/gains/' + name + '/d')
        i_clamp[name] = rospy.get_param('atlas_controller/gains/' + name + '/i_clamp')


def create_IK_solvers():
    global left_ik, right_ik
    global left_fk, right_fk
    global left_ikv, right_ikv
    global left_chain, right_chain

    with lock:

        # Define left and right arm chains
        left_chain = robot_kdl.getChain('utorso', 'l_hand')
        right_chain = robot_kdl.getChain('utorso', 'r_hand')

        # Define the left joint limits from the URDF
        left_min = kdl.JntArray(left_chain.getNrOfJoints())
        left_max = kdl.JntArray(left_chain.getNrOfJoints())
        for idx in xrange(0, left_chain.getNrOfJoints()):
            name = left_chain.getSegment(idx).getJoint().getName()
            left_min[idx] = robot_urdf.joints[name].limits.lower
            left_max[idx] = robot_urdf.joints[name].limits.upper
            
        # Define the right joint limits from the URDF
        right_min = kdl.JntArray(right_chain.getNrOfJoints())
        right_max = kdl.JntArray(right_chain.getNrOfJoints())
        for idx in xrange(0, right_chain.getNrOfJoints()):
            name = right_chain.getSegment(idx).getJoint().getName()
            right_min[idx] = robot_urdf.joints[name].limits.lower
            right_max[idx] = robot_urdf.joints[name].limits.upper

        # Initialize IK for left arm
        left_fk = kdl.ChainFkSolverPos_recursive(left_chain)
        left_ikv = kdl.ChainIkSolverVel_pinv(left_chain)
        left_ik = kdl.ChainIkSolverPos_NR_JL(left_chain, 
                                             left_min, left_max, 
                                             left_fk, left_ikv, 
                                             100, 1e-6)

        # Initialize IK for right arm
        right_fk = kdl.ChainFkSolverPos_recursive(right_chain)
        right_ikv = kdl.ChainIkSolverVel_pinv(right_chain)
        right_ik = kdl.ChainIkSolverPos_NR_JL(right_chain, 
                                              right_min, right_max,
                                              right_fk, right_ikv, 
                                              100, 1e-6)
    

def atlas_callback(atlas_msg):
    global left_chain, right_chain
    global left_q, right_q
    global left_desired, right_desired

    with lock:

        # Load gains if not already loaded
        load_gains(atlas_msg)

        # TODO: this can be optimized
        q = kdl.JntArray(left_chain.getNrOfJoints())
        for chain_idx in xrange(0, left_chain.getNrOfJoints()):
            
            # Get current joint name and use it to get joint index in state message
            name = left_chain.getSegment(chain_idx).getJoint().getName()
            tree_idx = atlas_msg.name.index(name)
            
            # Fill in corresponding joint index
            q[chain_idx] = atlas_msg.position[tree_idx]
            
        # Update global state 
        left_q = q
        if not left_desired:
            left_desired = left_q

        q = kdl.JntArray(right_chain.getNrOfJoints())
        for chain_idx in xrange(0, right_chain.getNrOfJoints()):
            
            # Get current joint name and use it to get joint index in state message
            name = right_chain.getSegment(chain_idx).getJoint().getName()
            tree_idx = atlas_msg.name.index(name)
            
            # Fill in corresponding joint index
            q[chain_idx] = atlas_msg.position[tree_idx]

        # Update global state 
        right_q = q
        if not right_desired:
            right_desired = right_q

def hydra_callback(hydra_msg):
    global tf, robot_urdf, robot_kdl
    global left_ik, right_ik
    global left_q, right_q
    global left_desired, right_desired
    global kp, ki, kd
    global pub

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
    
    with lock:

        # Create joint command for atlas
        n = len(atlasJointNames)
        command = AtlasCommand()
        command.position     = [0] * n
        command.velocity     = [0] * n
        command.effort       = [0] * n
        command.kp_position  = [ kp.get(name, 0.0) for name in atlasJointNames ]
        command.ki_position  = [ ki.get(name, 0.0) for name in atlasJointNames ]
        command.kd_position  = [ kd.get(name, 0.0) for name in atlasJointNames ]
        command.kp_velocity  = [0] * n
        command.i_effort_min = [ i_clamp.get(name, 0.0) for name in atlasJointNames ]
        command.i_effort_max = [ i_clamp.get(name, 0.0) for name in atlasJointNames ]
        command.k_effort     = [0] * n

        # Set the current left hand target position
        try:
            # Compute the relative transform to the hand
            left_time = tf.getLatestCommonTime("/utorso", "/hydra_left")
            left_pos, left_quat = tf.lookupTransform("/utorso", "/hydra_left", left_time)
            left_target = kdl.Frame(kdl.Vector(left_pos[0], left_pos[1], left_pos[2]))
            
            # Compute IK for desired joint position
            if hydra_msg.paddles[0].trigger > 0.9:
                left_desired = kdl.JntArray(left_q.rows())
                left_ret = left_ik.CartToJnt(left_q, left_target, left_desired);
                if (left_ret < 0):
                    raise Exception('Left inverse kinematics failed with ' + str(left_ret))

            # Fill in left command for atlas
            for idx in range(0, left_chain.getNrOfJoints()):
                name = left_chain.getSegment(idx).getJoint().getName()
                joint_idx = atlasJointNames.index(name)
                command.position[joint_idx] = left_desired[idx]
                command.k_effort[joint_idx] = 255

        except Exception as e:
            rospy.logwarn('Left hand failed: %s', str(e))

        # Set the current right hand target position
        try:
            # Compute the relative transform to the hand
            right_time = tf.getLatestCommonTime("/utorso", "/hydra_right")
            right_pos, right_quat = tf.lookupTransform("/utorso", "/hydra_right", right_time)
            right_target = kdl.Frame(kdl.Vector(right_pos[0], right_pos[1], right_pos[2]))
            
            # Compute IK for desired joint position
            if hydra_msg.paddles[1].trigger > 0.9:
                right_desired = kdl.JntArray(right_q.rows())
                right_ret = right_ik.CartToJnt(right_q, right_target, right_desired);
                if (right_ret < 0):
                    raise Exception('Right inverse kinematics failed with ' + str(right_ret))

            # Fill in right command for atlas
            for idx in range(0, right_chain.getNrOfJoints()):
                name = right_chain.getSegment(idx).getJoint().getName()
                joint_idx = atlasJointNames.index(name)
                command.position[joint_idx] = right_desired[idx]
                command.k_effort[joint_idx] = 255

        except Exception as e:
            rospy.logwarn('Right hand failed: %s', str(e))

        # Send command to atlas!
        pub.publish(command)
        

def main():
    global tf, robot_urdf, robot_kdl, pub

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

    # Publish Atlas commands
    pub = rospy.Publisher('/atlas/atlas_command', AtlasCommand)

    # Start main event handling loop
    rospy.loginfo('Started kdl_hydra node...')
    rospy.spin()
    rospy.loginfo('Stopping kdl_hydra node...')


if __name__ == '__main__':
    main()
