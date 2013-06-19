#! /usr/bin/env python
import roslib; roslib.load_manifest('super_teleop')

from razer_hydra.msg import Hydra
from std_msgs.msg import String, Empty
from atlas_replay.srv import Record, RecordRequest, Play, PlayRequest
import tf
import rospy

PADDLE_NAMES = [ 'hydra_left', 'hydra_right' ]
BUTTON_NAMES = [ 'trigger', '1', '2', '3', '4', 'start', 'stick' ]

class HydraControl():

    def status(self, message):
        msg = String()
        msg.data = message
        self.commands.publish(msg)

    def defaultstatus(self):
        self.status("text:setText('[%s] %s')" %
                    (str(self.slot) if self.slot != 0 else "[ERASE]",
                     "REC" if self.record_msg.record else "---"))

    def init(self):
        self.commands = rospy.Publisher("commands", String, tcp_nodelay=True)
        self.br = tf.TransformBroadcaster()
        self.prev_msg = None
        self.slot = 0

        self.record = rospy.ServiceProxy('record', Record)
        self.send = rospy.ServiceProxy('send', Play)
        self.play = rospy.ServiceProxy('play', Play)

        self.record_msg = RecordRequest()
        self.input = rospy.Subscriber("hydra_calib", Hydra, self.process_hydra)
        self.snapshot = rospy.Publisher("snapshot_request", Empty, tcp_nodelay=True)

        # These are available in simulation only.
        self.mode = rospy.Publisher('/atlas/mode', 
                                    String, None, False, True, None)
        self.control_mode = rospy.Publisher('/atlas/control_mode', 
                                            String, None, False, True, None)

    
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
                self.status("text:setText('Executing current.')")
                print "Executing current trajectory."
                send_msg = PlayRequest()
                send_msg.slots = [0, 1] # don't save trajectory, execute immediately
                self.send(send_msg)
                print "Execution complete."
            except rospy.ServiceException, e:
                print "Execution command failed: %s" % str(e)

        if left.buttons[6] and not left_old.buttons[6] and not self.record_msg.record:
            try:
                if self.slot == 0:
                    self.status("text:setText('Erasing [%d]')" % self.slot)
                    print "Erasing trajectory."
                    send_msg = PlayRequest()
                    send_msg.slots = [] # don't save trajectory, don't execute (erase)
                    self.send(send_msg)
                    print "Trajectory erased."
                else:
                    self.status("text:setText('Saving [%d]')" % self.slot)
                    print "Saving trajectory to slot [%d]." % self.slot
                    send_msg = PlayRequest()
                    send_msg.slots = [self.slot] # save trajectory, do not execute
                    self.send(send_msg)
                    print "Saved trajectory."
            except rospy.ServiceException, e:
                print "Execution command failed: %s" % str(e)

        if left.joy[1] > 0.9 and left_old.joy[1] <= 0.9:
            if self.slot >= 255:
                self.slot = 0
            else:
                self.slot = self.slot + 1;
            print "Changed active slot to [%d]" % self.slot
            self.defaultstatus()

        if left.joy[1] < -0.9 and left_old.joy[1] >= -0.9:
            if self.slot <= 0:
                self.slot = 255
            else:
                self.slot = self.slot - 1;
            print "Changed active slot to [%d]" % self.slot
            self.defaultstatus()

        if not self.record_msg.record:
            changed = False

            if left.buttons[1] and not left_old.buttons[1]:
                self.record_msg.left_leg = not self.record_msg.left_leg
                changed = True

            if left.buttons[2] and not left_old.buttons[2]:
                self.record_msg.right_leg = not self.record_msg.right_leg
                changed = True

            if left.buttons[3] and not left_old.buttons[3]:
                self.record_msg.left_arm = not self.record_msg.left_arm
                changed = True

            if left.buttons[4] and not left_old.buttons[4]:
                self.record_msg.right_arm = not self.record_msg.right_arm
                changed = True
            
            if left.buttons[5] and not left_old.buttons[5]:
                self.record_msg.torso = not self.record_msg.torso
                changed = True

            # Send update to atlas_replay
            if changed:
                self.defaultstatus()
                try:
                    self.record(self.record_msg)
                except rospy.ServiceException, e:
                    print "Update command failed: %s" % str(e)
                       
        if left.trigger > 0.9 and left_old.trigger <= 0.9:
            self.record_msg.record = not self.record_msg.record
            try:
                self.record(self.record_msg)
            except rospy.ServiceException, e:
                print "Recording command failed: %s" % str(e)
            self.defaultstatus()

        if right.trigger > 0.9 and right_old.trigger <= 0.9:
            print "Requesting snapshot."
            self.snapshot.publish(Empty());

        if right.buttons[0] and not right_old.buttons[0]:
            print "Resetting to standing..."
            self.mode.publish("harnessed")
            self.control_mode.publish("Freeze")
            self.control_mode.publish("StandPrep")
            rospy.sleep(2.0)
            self.mode.publish("nominal")
            rospy.sleep(0.3)
            self.control_mode.publish("Stand")
            print "Reset complete."

if __name__ == '__main__':
    rospy.init_node('hydra_control')
    hydra_ctrl = HydraControl()
    hydra_ctrl.run()

