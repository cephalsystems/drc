#! /usr/bin/env python
import roslib; roslib.load_manifest('super_teleop')

from atlas_msgs.msg import AtlasState, AtlasCommand
from razer_hydra.msg import Hydra

import rospy

ATLAS_JOINT_NAMES = [ 'back_lbz', 'back_mby', 'back_ubx', 'neck_ay',
                      'l_leg_uhz', 'l_leg_mhx', 'l_leg_lhy', 'l_leg_kny', 'l_leg_uay', 'l_leg_lax',
                      'r_leg_uhz', 'r_leg_mhx', 'r_leg_lhy', 'r_leg_kny', 'r_leg_uay', 'r_leg_lax',
                      'l_arm_usy', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx',
                      'r_arm_usy', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx']

PADDLE_NAMES = [ 'left', 'right' ]
BUTTON_NAMES = [ 'trigger', '1', '2', '3', '4', 'start', 'stick' ]

class HydraRecorder:
    def __init__(self):

        # Initialize an Atlas command structure
        n = 28
        self.command = AtlasCommand()
        self.command.position    = [0] * n
        self.command.velocity    = [0] * n
        self.command.effort      = [0] * n
        self.command.kp_velocity = [0] * n
        self.command.k_effort    = [255] * n
        self.command.kp_position = [ rospy.get_param('atlas_controller/gains/' + name + '/p')
                                     for name in ATLAS_JOINT_NAMES ] 
        self.command.ki_position = [ rospy.get_param('atlas_controller/gains/' + name + '/i') 
                                     for name in ATLAS_JOINT_NAMES ] 
        self.command.kd_position = [ rospy.get_param('atlas_controller/gains/' + name + '/d') 
                                     for name in ATLAS_JOINT_NAMES ] 
    
        # Initialize empty tables for arms and legs
        self.arm_table = [ None ] * 14
        self.leg_table = [ None ] * 14

        # Connects to necessary command topics
        self.cmd_pub = rospy.Publisher("/atlas/atlas_command", AtlasCommand)
        rospy.Subscriber("/atlas/atlas_state", AtlasState, self.atlas_callback, queue_size=1)
        
        rospy.Subscriber("/arms/hydra_calib", Hydra, self.hydra_arms_callback, queue_size=1)
        rospy.Subscriber("/legs/hydra_calib", Hydra, self.hydra_legs_callback, queue_size=1)


    def atlas_callback(self, msg):
        # Update current position array
        self.position = msg.position

    
    def hydra_arms_callback(self, msg):
        self.hydra_callback(self.arm_table, msg)

        
    def hydra_legs_callback(self, msg):
        self.hydra_callback(self.leg_table, msg)

        
    def hydra_callback(self, table, msg):

        # When a button is pressed while the trigger is held, record the position
        for (i, paddle) in enumerate(msg.paddles):
            if paddle.trigger > 0.9:
                for (j, button) in enumerate(paddle.buttons):
                    if button and self.position:
                        print 'Recording ' + PADDLE_NAMES[i] + ':' + BUTTON_NAMES[j]
                        table[i * len(paddle.buttons) + j] = self.position
            else:
                for (j, button) in enumerate(paddle.buttons):
                    if button and table[i * len(paddle.buttons) + j]:
                        print 'Playing ' + PADDLE_NAMES[i] + ':' + BUTTON_NAMES[j]
                        self.command.position = table[i * len(paddle.buttons) + j]
                        self.cmd_pub.publish(self.command)
                        return


if __name__ == '__main__':
    rospy.init_node('hydra_recorder')
    hr = HydraRecorder()
    rospy.spin()
    print "TABLE:"
    print hr.arm_table
    print hr.leg_table

