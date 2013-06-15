#! /usr/bin/env python
import roslib; roslib.load_manifest('super_teleop')

from razer_hydra.msg import Hydra
from std_msgs.msg import String
import tf
import rospy

PADDLE_NAMES = [ 'hydra_left', 'hydra_right' ]

class HydraControl():

    def init(self):
        self.input = rospy.Subscriber("hydra_calib_throttle", Hydra, self.process_hydra)
        self.status = rospy.Publisher("record_state", String, tcp_nodelay=True)
        self.br = tf.TransformBroadcaster()
        self.limbs = { torso: false,
                       left_arm: false,
                       right_arm: false,
                       left_leg: false,
                       right_leg: false }
    
    def run(self):
        self.init()
        rospy.spin()

    def process_hydra(self, msg):
        if not self.prev_msg:
            self.prev_msg = msg
            return

        # Left paddle bindings
        left_new = msg.paddles[0]
        left_old = prev_msg.paddles[0]

        if left.buttons[0] and not left.buttons[0]:
            pass

        if left.buttons[1] and not left.buttons[1]:
            self.limbs['left_leg'] = True

        if left.buttons[2] and not left.buttons[2]:
            self.limbs['right_leg'] = True

        if left.buttons[3] and not left.buttons[3]:
            self.limbs['left_arm'] = True

        if left.buttons[4] and not left.buttons[4]:
            self.limbs['right_arm'] = True

        if left.buttons[6] and not left.buttons[6]:
            self.limbs['torso'] = True
                       
        if left.buttons[7] and not left.buttons[7]:
            pass

        # Publish current state of system
        status.publish(str(self.limbs))

if __name__ == '__main__':
    rospy.init_node('hydra_control')
    hydra_ctrl = HydraControl()
    hydra_ctrl.run()

