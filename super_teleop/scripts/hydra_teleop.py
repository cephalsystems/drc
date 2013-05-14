#! /usr/bin/env python
import roslib; roslib.load_manifest('super_teleop')

from atlas_msgs.msg import AtlasSimInterfaceCommand, \
                           AtlasBehaviorStepData, \
                           AtlasBehaviorStepParams, \
                           AtlasBehaviorStandParams, \
                           AtlasBehaviorManipulateParams
from std_msgs.msg import Header
from razer_hydra.msg import Hydra

from geometry_msgs.msg import Pose
from std_msgs.msg import String

import tf
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

import math
import rospy


PADDLE_NAMES = [ 'left', 'right' ]
STEP_NAMES = [ 'step1', 'step2', 'step3', 'step4' ]

class AtlasTeleop():
    
    # Walking parameters
    params = {"Step Height":{"value":0, "min":-1, "max":1, "type":"float"},
              "Stride Duration":{ "value":0.63, "min": 0, "max":100, \
                                "type":"float"},
              "Swing Height":{"value":0.3, "min":0, "max":1, "type":"float"}}

    steps = []

    
    def init(self):
        self._isPressing = False;
        
        # Connects to necessary command topics
        self.command = rospy.Publisher('atlas/atlas_sim_interface_command', \
          AtlasSimInterfaceCommand)
        self.mode = rospy.Publisher('/atlas/mode', String, None, False, \
          True, None)
        self.control_mode = rospy.Publisher('/atlas/control_mode', \
          String, None, False, True, None)

        # Listen for hydra messages
        rospy.Subscriber("/arms/hydra_calib", Hydra, self.process_hydra)
    

    def run(self):
        self.init()
        br = tf.TransformBroadcaster()

        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            for (i, step) in enumerate(self.steps):
                br.sendTransform([ step.pose.position.x,
                                   step.pose.position.y,
                                   step.pose.position.z ],
                                 [ step.pose.orientation.w,
                                   step.pose.orientation.x,
                                   step.pose.orientation.y,
                                   step.pose.orientation.z ],
                                 rospy.Time.now(),
                                 STEP_NAMES[i],
                                 "pelvis")
            for i in xrange(len(self.steps), 4):
                br.sendTransform([ 0, 0, 0 ],
                                 [ 1, 0, 0, 0 ],
                                 rospy.Time.now(),
                                 STEP_NAMES[i],
                                 "pelvis")
            r.sleep()


    def process_hydra(self, msg):

        # Drive the hydra around to place footsteps
        for (i, paddle) in enumerate(msg.paddles):
            if paddle.buttons[2] and len(self.steps) < 4 and not self._isPressing:
                self._isPressing = True
                rospy.loginfo('Footstep added: ' + PADDLE_NAMES[i])
                self.add_step(i, paddle.transform)
                return

        # Undo a step if some button is pressed
        for (i, paddle) in enumerate(msg.paddles):
            if paddle.buttons[4] and len(self.steps) > 0 and not self._isPressing:
                self._isPressing = True
                rospy.loginfo('Last footstep removed.')
                del self.steps[-1]

        # If we are happy with the footsteps then walk
        for (i, paddle) in enumerate(msg.paddles):
            if paddle.buttons[0] and len(self.steps) == 4 and not self._isPressing:
                self._isPressing = True
                rospy.loginfo('Walking!')
                self.walk()
                return

        # RESET robot if necessary
        if msg.paddles[0].buttons[5] and not self._isPressing:
                self._isPressing = True
                rospy.loginfo('Harnessing robot...')
                self.reset_to_harnessed()
                rospy.loginfo('Harnessing complete.')
        elif msg.paddles[1].buttons[5] and not self._isPressing:
                self._isPressing = True
                rospy.loginfo('Resetting robot...')
                self.reset_to_standing()
                rospy.loginfo('Resetting complete.')

        # If no buttons are pushed, clear the debouncing
        if not any(msg.paddles[0].buttons) and not any(msg.paddles[1].buttons):
            self._isPressing = False


    # Publishes commands to reset robot to a standing position
    def reset_to_harnessed(self):
        self.mode.publish("harnessed")
        rospy.sleep(0.3)


    def reset_to_standing(self):
        self.mode.publish("harnessed")
        self.control_mode.publish("Freeze")
        self.control_mode.publish("StandPrep")
        rospy.sleep(2.0)
        self.mode.publish("nominal")
        rospy.sleep(0.3)
        self.control_mode.publish("Stand")


    # Construct a new step from the hydra's projected ground position
    def add_step(self, is_right_foot, transform):
        
        # Create a new step object
        step = AtlasBehaviorStepData()

        # Bookkeeping for step and foot
        step.step_index = len(self.steps)
        step.foot_index = is_right_foot
        step.duration = self.params["Stride Duration"]["value"]
            
        # Project foot position into XY plane
        step.pose.position.x = transform.translation.x
        step.pose.position.y = transform.translation.y
        step.pose.position.z = self.params["Step Height"]["value"]
         
        # Project rotation into XY plane
        (r, p, y) = euler_from_quaternion([ transform.rotation.w, 
                                            transform.rotation.x,
                                            transform.rotation.y,
                                            transform.rotation.z ],
                                            axes='sxyz')
        Q = quaternion_from_euler(0, 0, y)

        step.pose.orientation.x = Q[0]
        step.pose.orientation.y = Q[1]
        step.pose.orientation.z = Q[2]
        step.pose.orientation.w = Q[3]

        # Hard coded swing height
        step.swing_height = self.params["Swing Height"]["value"]

        # Add this step onto the list
        self.steps.append(step)

        
    # Builds a trajectory of step commands. 
    def walk(self):
        walk_goal = AtlasSimInterfaceCommand()
        walk_goal.behavior = AtlasSimInterfaceCommand.WALK
        walk_goal.walk_params.use_demo_walk = False

        # 0 for full BDI control, 255 for PID control
        walk_goal.k_effort =  [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
               
        # Load in the step array 
        walk_goal.walk_params.step_data = self.steps
        
        # Send command to Atlas
        print(str(walk_goal))
        self.command.publish(walk_goal)
        self.steps = []

            
if __name__ == '__main__':
    rospy.init_node('walking_client')
    teleop = AtlasTeleop()
    teleop.run()
