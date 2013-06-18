#! /usr/bin/env python
import roslib; roslib.load_manifest('super_teleop')

from razer_hydra.msg import Hydra
from std_msgs.msg import String
from atlas_replay.srv import Record, RecordRequest, Play, PlayRequest
import tf
import rospy

PADDLE_NAMES = [ 'hydra_left', 'hydra_right' ]
BUTTON_NAMES = [ 'trigger', '1', '2', '3', '4', 'start', 'stick' ]

class HydraControl():

    def init(self):
        self.status = rospy.Publisher("record_state", String, tcp_nodelay=True)
        self.br = tf.TransformBroadcaster()
        self.prev_msg = None

        print 'Waiting for record service'
        rospy.wait_for_service('record')
        self.record = rospy.ServiceProxy('record', Record)

        print 'Waiting for send service'
        rospy.wait_for_service('send')
        self.send = rospy.ServiceProxy('send', Play)

        print 'Waiting for play service'
        rospy.wait_for_service('play')
        self.play = rospy.ServiceProxy('play', Play)

        self.record_msg = RecordRequest()
        self.input = rospy.Subscriber("hydra_calib", Hydra, self.process_hydra)
    
    def run(self):
        self.init()
        print 'Starting hydra control...'
        rospy.spin()
        print 'Stopping hydra control...'

    def process_hydra(self, msg):
        if not self.prev_msg:
            self.prev_msg = msg
            return

        # Get paddle states and overwrite prev message
        left = msg.paddles[0]
        right = msg.paddles[1]
        left_old = self.prev_msg.paddles[0]
        right_old = self.prev_msg.paddles[1]
        self.prev_msg = msg

        # Left paddle bindings
        if left.buttons[0] and not left_old.buttons[0] and not self.record_msg.record:
            try:
                print "Executing current trajectory."
                send_msg = PlayRequest()
                send_msg.slots = [0, 1] # don't save trajectory, execute immediately
                self.send(send_msg)
                print "Execution complete."
            except rospy.ServiceException, e:
                print "Execution command failed: %s" % str(e)

        if left.buttons[6] and not left_old.buttons[6] and not self.record_msg.record:
            try:
                print "Erasing trajectory."
                send_msg = PlayRequest()
                send_msg.slots = [] # don't save trajectory, don't execute (erase)
                self.send(send_msg)
                print "Trajectory erased."
            except rospy.ServiceException, e:
                print "Execution command failed: %s" % str(e)

        if not self.record_msg.record:
            if left.buttons[1] and not left_old.buttons[1]:
                self.record_msg.left_leg = not self.record_msg.left_leg

            if left.buttons[2] and not left_old.buttons[2]:
                self.record_msg.right_leg = not self.record_msg.right_leg

            if left.buttons[3] and not left_old.buttons[3]:
                self.record_msg.left_arm = not self.record_msg.left_arm

            if left.buttons[4] and not left_old.buttons[4]:
                self.record_msg.right_arm = not self.record_msg.right_arm
            
            if left.buttons[5] and not left_old.buttons[5]:
                self.record_msg.torso = not self.record_msg.torso
                       
        if left.trigger > 0.9 and left_old.trigger <= 0.9:
            self.record_msg.record = not self.record_msg.record
            try:
                self.record(self.record_msg)
            except rospy.ServiceException, e:
                print "Recording command failed: %s" % str(e)

        # Publish current state of system and record old state
        self.status.publish(str(self.record_msg))

if __name__ == '__main__':
    rospy.init_node('hydra_control')
    hydra_ctrl = HydraControl()
    hydra_ctrl.run()

