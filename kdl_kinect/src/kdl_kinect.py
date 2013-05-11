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
from pi_tracker.msg import Skeleton
from atlas_msgs.msg import AtlasState
from atlas_msgs.msg import AtlasCommand

from teleop_limb import TeleopLimb
import threading, time

SKELETON_JOINT_NAMES = ['head', 'neck', 'torso', 
                        'left_shoulder', 'left_elbow', 'left_hand', 
                        'right_shoulder', 'right_elbow', 'right_hand', 
                        'left_hip', 'left_knee', 'left_foot', 
                        'right_hip', 'right_knee', 'right_foot']

ATLAS_JOINT_NAMES = [ 'back_lbz', 'back_mby', 'back_ubx', 'neck_ay',
                      'l_leg_uhz', 'l_leg_mhx', 'l_leg_lhy', 'l_leg_kny', 'l_leg_uay', 'l_leg_lax',
                      'r_leg_uhz', 'r_leg_mhx', 'r_leg_lhy', 'r_leg_kny', 'r_leg_uay', 'r_leg_lax',
                      'l_arm_usy', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx',
                      'r_arm_usy', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx']

pub = None
limbs = None
command = initAtlasCommand()


def initAtlasCommand():
    n = 28
    command = AtlasCommand()
    command.position    = [0] * n
    command.velocity    = [0] * n
    command.effort      = [0] * n
    command.kp_position = [0] * n
    command.ki_position = [0] * n
    command.kd_position = [0] * n
    command.kp_velocity = [0] * n
    command.k_effort    = [0] * n
    return command


def skeleton_callback(skeleton_msg):
    global limbs
        

def atlas_callback(atlas_msg):
    global pub
    global limbs
    global command

    # Update joint values in limbs
    for limb in limbs.values():
        limb.update(atlas_msg)
        
    # Send out current atlas command
    pub.publish(command)


def hydra_arms_callback(hydra_msg):
    global limbs, frames
    global command

    # Left arm update
    if hydra_msg.paddles[0].trigger > 0.9:
        try:
            limbs['left_arm'].solve('/arms/left')
            limbs['left_arm'].populate(command)
        except Exception as e:
            rospy.logwarn('Left arm failed: %s', str(e))
    else:
        limbs['left_arm'].clear(command)

    # Right arm update
    if hydra_msg.paddles[1].trigger > 0.9:
        try:
            limbs['right_arm'].solve('/arms/right')
            limbs['right_arm'].populate(command)
        except Exception as e:
            rospy.logwarn('Right arm failed: %s', str(e))
    else:
        limbs['right_arm'].clear(command)


def hydra_legs_callback(hydra_msg):
    global limbs, frames
    global command

    # Left leg update
    if hydra_msg.paddles[0].trigger > 0.9:
        try:
            limbs['left_leg'].solve('/legs/left')
            limbs['left_leg'].populate(command)
        except Exception as e:
            rospy.logwarn('Left leg failed: %s', str(e))
    else:
        limbs['left_leg'].clear(command)

    # Right leg update
    if hydra_msg.paddles[1].trigger > 0.9:
        try:
            limbs['right_leg'].solve('/legs/right')
            limbs['right_leg'].populate(command)
        except Exception as e:
            rospy.logwarn('Right leg failed: %s', str(e))
    else:
        limbs['right_leg'].clear(command)

def main():
    global tf
    global command, limbs
    global pub

    # Initialize the ROS node
    rospy.init_node('kdl_kinect')

    # Create a transform listener
    tf = TransformListener()
    
    # Retrieve raw robot parameters from rosmaster
    robot_string = rospy.get_param("robot_description", None)
    if not robot_string:
        raise Exception('Robot model not specified')

    # Load URDF model of robot description locally
    robot_urdf = URDF.parse_xml_string(robot_string)

    # Load URDF model of robot description into KDL
    robot_kdl = kdl_parser.kdl_tree_from_urdf_model(robot_urdf)

    # Publish Atlas commands
    pub = rospy.Publisher('/atlas/atlas_command', AtlasCommand)

    # Subscribe to hydra and atlas updates
    rospy.Subscriber("/arms/hydra_calib", Hydra, hydra_callback, queue_size = 1)
    rospy.Subscriber("/skeleton", Skeleton, skeleton_callback, queue_size = 1)
    rospy.Subscriber("/atlas/atlas_state", AtlasState, atlas_callback, queue_size = 1)

    # Start main event handling loop
    rospy.loginfo('Started kdl_hydra node...')
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        r.sleep()
    rospy.loginfo('Stopping kdl_hydra node...')


if __name__ == '__main__':
    main()
