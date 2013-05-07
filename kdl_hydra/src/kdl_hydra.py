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


def hydra_callback(hydra_msg):
    pass

def main():
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
