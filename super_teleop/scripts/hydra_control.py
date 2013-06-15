#! /usr/bin/env python
import roslib; roslib.load_manifest('super_teleop')

from razer_hydra.msg import Hydra
from std_msgs.msg import String
from atlas_replay.srv import Record, Play
import tf
import rospy

PADDLE_NAMES = [ 'hydra_left', 'hydra_right' ]

class HydraControl():

    def init(self):
        self.input = rospy.Subscriber("hydra_calib_throttle", Hydra, self.process_hydra)
        self.status = rospy.Publisher("record_state", String, tcp_nodelay=True)
        self.br = tf.TransformBroadcaster()

        rospy.log_info('Waiting for record service')
        rospy.wait_for_service('/record')
        self.record = rospy.ServiceProxy('/record', Record)

        rospy.log_info('Waiting for send service')
        rospy.wait_for_service('/send')
        self.send = rospy.ServiceProxy('/send', Play)

        rospy.log_info('Waiting for play service')
        rospy.wait_for_service('/play')
        self.play = rospy.ServiceProxy('/play', Play)

        self.record_msg = Record()
    
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

        if left.buttons[0] and not left.buttons[0] and not self.record_msg.record:
            try:
                print "Executing current trajectory"
                send_msg = Play()
                send_msg.slots = [0, 1] # don't save trajectory, execute immediately
                self.send(send_msg)
            except rospy.ServiceException, e:
                print "Execution command failed: %s" % str(e)

        if left.buttons[1] and not left.buttons[1]:
            self.record_msg.left_leg = not self.record_msg.left_leg

        if left.buttons[2] and not left.buttons[2]:
            self.record_msg.right_leg = not self.record_msg.right_leg

        if left.buttons[3] and not left.buttons[3]:
            self.record_msg.left_arm = not self.record_msg.left_arm

        if left.buttons[4] and not left.buttons[4]:
            self.record_msg.right_arm = not self.record_msg.right_arm
            
        if left.buttons[6] and not left.buttons[6]:
            self.record_msg.torso = not self.record_msg.torso
                       
        if left.buttons[7] and not left.buttons[7]:
            self.record_msg.record = not self.record_msg.record
            try:
                print "Recording: %s" % str(self.record_msg.record)
                self.record(self.record_msg)
            except rospy.ServiceException, e:
                print "Recording command failed: %s" % str(e)

        # Publish current state of system
        status.publish(str(self.record_msg))

if __name__ == '__main__':
    rospy.init_node('hydra_control')
    hydra_ctrl = HydraControl()
    hydra_ctrl.run()

