#!/usr/bin/env python
#
# Read from two TF frames and attempt to create joint solutions that put the end
# effectors at those TF frames.
#

import roslib; roslib.load_manifest('kdl_hydra')
import rospy

from tf import TransformListener
from razer_hydra.msg import Hydra
from pykdl_utils import kdl_parser
from urdf_parser_py.urdf import URDF
import PyKDL as kdl

tf = None
robot_urdf = None
robot_kdl = None

def hydra_callback(hydra_msg):
    global tf, robot_urdf, robot_kdl

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

    # Get the current right hand target position
    try:
        left_time = tf.getLatestCommonTime("/utorso", "/hydra_left")
        left_pos, left_quat = tf.lookupTransform("/utorso", "/hydra_left", left_time)
    except Exception as e:
        rospy.logwarn('Unable to compute left hand target: %s', str(e))

    # Get the current left hand target position
    try:
        right_time = tf.getLatestCommonTime("/utorso", "/hydra_right")
        right_pos, right_quat = tf.lookupTransform("/utorso", "/hydra_right", right_time)
    except Exception as e:
        rospy.logwarn('Unable to compute left hand target: %s', str(e))

    # Compute IK solution to put arm at location
    # TODO: fill this in

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

    # Subscribe to hydra updates
    rospy.Subscriber("hydra_calib", Hydra, hydra_callback)

    # Start main event handling loop
    rospy.loginfo('Started kdl_hydra node...')
    rospy.spin()
    rospy.loginfo('Stopping kdl_hydra node...')


if __name__ == '__main__':
    main()
