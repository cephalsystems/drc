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
from atlas_msgs.msg import AtlasState
from atlas_msgs.msg import AtlasCommand

from teleop_limb import TeleopLimb
import threading

pub = None
limbs = {}
command = None

def init_command():
    global command

    # Create empty Atlas command
    command = AtlasCommand()
    n = len(command.position)
    command.position     = [0] * n
    command.velocity     = [0] * n
    command.effort       = [0] * n
    command.kp_position  = [0] * n
    command.ki_position  = [0] * n
    command.kd_position  = [0] * n
    command.kp_velocity  = [0] * n
    command.i_effort_min = [0] * n
    command.i_effort_max = [0] * n
    command.k_effort     = [0] * n


def atlas_callback(atlas_msg):
    global pub
    global limbs
    global command

    # Update joint values in limbs
    for (name, limb) in limbs:
        limb.update(atlas_msg)
        
    # Send out current atlas command
    pub.publish(command)


def hydra_arms_callback(hydra_msg):
    global limbs, frames
    global command

    # Left arm update
    if hydra_msg.paddles[0].trigger > 0.9:
        try:
            limb['left_arm'].solve('/arms/hydra_left')
            limb.populate(command)
        except Exception as e:
            rospy.logwarn('Left arm failed: %s', str(e))

    # Right arm update
    if hydra_msg.paddles[1].trigger > 0.9:
        try:
            limb['right_arm'].solve('/arms/hydra_right')
            limb.populate(command)
        except Exception as e:
            rospy.logwarn('Right arm failed: %s', str(e))

def hydra_legs_callback(hydra_msg):
    global limbs, frames
    global command

    # Left leg update
    if hydra_msg.paddles[0].trigger > 0.9:
        try:
            limb['left_leg'].solve('/legs/hydra_left')
            limb.populate(command)
        except Exception as e:
            rospy.logwarn('Left leg failed: %s', str(e))

    # Right leg update
    if hydra_msg.paddles[1].trigger > 0.9:
        try:
            limb['right_leg'].solve('/legs/hydra_right')
            limb.populate(command)
        except Exception as e:
            rospy.logwarn('Right leg failed: %s', str(e))


def main():
    global tf, robot_urdf, robot_kdl, limbs
    global pub

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

    # Create limbs!
    limbs = {
        'left_arm': TelopLimb(tf, robot_urdf, robot_kdl, 'utorso', 'l_hand'),
        'right_arm': TelopLimb(tf, robot_urdf, robot_kdl, 'utorso', 'r_hand'),
        'left_leg': TelopLimb(tf, robot_urdf, robot_kdl, 'utorso', 'l_foot'),
        'right_leg': TelopLimb(tf, robot_urdf, robot_kdl, 'utorso', 'r_foot'),
        }
    
    # Subscribe to hydra and atlas updates
    rospy.Subscriber("/arms/hydra_calib", Hydra, hydra_arms_callback)
    rospy.Subscriber("/legs/hydra_calib", Hydra, hydra_legs_callback)
    rospy.Subscriber("/atlas/atlas_state", AtlasState, atlas_callback)

    # Publish Atlas commands
    pub = rospy.Publisher('/atlas/atlas_command', AtlasCommand)

    # Start main event handling loop
    rospy.loginfo('Started kdl_hydra node...')
    rospy.spin()
    rospy.loginfo('Stopping kdl_hydra node...')


if __name__ == '__main__':
    main()
