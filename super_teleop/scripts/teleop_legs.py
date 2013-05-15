#! /usr/bin/env python
import roslib; roslib.load_manifest('super_teleop')

from atlas_msgs.msg import AtlasSimInterfaceCommand, \
                           AtlasSimInterfaceState, \
                           AtlasBehaviorStepData, \
                           AtlasBehaviorWalkParams, \
                           AtlasBehaviorStandParams
from std_msgs.msg import Header
from razer_hydra.msg import Hydra

from geometry_msgs.msg import Pose
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker

import tf
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

import math
import rospy
import itertools

PADDLE_NAMES = [ 'hydra_left', 'hydra_right' ]
NUM_STEPS = 7


def grouper(n, iterable, fillvalue=None):
    "grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"
    args = [iter(iterable)] * n
    return itertools.izip_longest(*args, fillvalue=fillvalue)


class AtlasTeleop():
    
    # Walking parameters
    params = {"Step Height":{"value":0, "min":-1, "max":1, "type":"float"},
              "Stride Duration":{ "value":0.63, "min": 0, "max":100, \
                                "type":"float"},
              "Swing Height":{"value":0.3, "min":0, "max":1, "type":"float"}}

    steps = []

    
    def init(self):
        self._isPressing = False;
        self._isWalking = False;
        
        # Connects to necessary command topics
        self.command = rospy.Publisher('/atlas/atlas_sim_interface_command', 
                                       AtlasSimInterfaceCommand)
        self.mode = rospy.Publisher('/atlas/mode', String, None, False, 
                                    True, None)
        self.control_mode = rospy.Publisher('/atlas/control_mode', 
                                            String, None, False, True, None)
        self.markers = rospy.Publisher('/steps', MarkerArray)
        self.feet = rospy.Publisher('/feet', MarkerArray)

        # Connect to TF
        self.tf = tf.TransformListener()

        # Initialize foot markers
        self.foot_markers = MarkerArray()

        left_foot = Marker()
        left_foot.header.frame_id = "world"
        left_foot.header.stamp = rospy.Time.now()
        left_foot.ns = "foot"
        left_foot.id = 0
        left_foot.action = Marker.ADD
        left_foot.type = Marker.MESH_RESOURCE
        left_foot.mesh_resource = "package://atlas/meshes/l_foot.dae"
        left_foot.lifetime = rospy.Duration.from_sec(0.2)
        left_foot.scale.x = 1.0
        left_foot.scale.y = 1.0
        left_foot.scale.z = 1.0
        left_foot.color.r = 0.5
        left_foot.color.g = 0.5
        left_foot.color.b = 1.0
        left_foot.color.a = 1.0
        self.foot_markers.markers.append(left_foot)

        right_foot = Marker()
        right_foot.header.frame_id = "world"
        right_foot.header.stamp = rospy.Time.now()
        right_foot.ns = "foot"
        right_foot.id = 1
        right_foot.action = Marker.ADD
        right_foot.type = Marker.MESH_RESOURCE
        right_foot.mesh_resource = "package://atlas/meshes/r_foot.dae" 
        right_foot.lifetime = rospy.Duration.from_sec(0.2)
        right_foot.scale.x = 1.0
        right_foot.scale.y = 1.0
        right_foot.scale.z = 1.0
        right_foot.color.r = 1.0
        right_foot.color.g = 0.5
        right_foot.color.b = 0.5
        right_foot.color.a = 1.0
        self.foot_markers.markers.append(right_foot)

        # Wait for transforms
        self.tf.waitForTransform("world", PADDLE_NAMES[0], rospy.Time.now(), rospy.Duration(10.0))
        self.tf.waitForTransform("world", PADDLE_NAMES[1], rospy.Time.now(), rospy.Duration(10.0))
        rospy.loginfo('Starting up leg teleop...')

        # Listen for hydra messages
        rospy.Subscriber("hydra_calib", Hydra, self.process_hydra)
        rospy.Subscriber("/atlas/atlas_sim_interface_state", AtlasSimInterfaceState, self.process_atlas)


    def run(self):
        self.init()
        step_markers = MarkerArray()

        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            step_markers.markers = []

            for (i, step) in enumerate(self.steps):
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "step"
                marker.id = i
                marker.action = Marker.ADD
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = \
                    "package://atlas/meshes/r_foot.dae" \
                    if step.foot_index else \
                    "package://atlas/meshes/l_foot.dae"
                marker.lifetime = rospy.Duration.from_sec(0.1)
                marker.pose = step.pose
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.r = 0.0
                marker.color.g = 0.5
                marker.color.b = 0.5
                marker.color.a = 1.0
                step_markers.markers.append(marker)

            self.markers.publish(step_markers)
            r.sleep()


    def process_hydra(self, msg):
        feet = [ None, None ]

        # Drive the hydra around to place footsteps
        for (i, paddle) in enumerate(msg.paddles):
            try:
                (trans,rot) = self.tf.lookupTransform('world', PADDLE_NAMES[i], rospy.Time(0))
                feet[i] = self.compute_step(i, trans, rot)
            except (tf.LookupException, 
                    tf.ConnectivityException, 
                    tf.ExtrapolationException) as e:
                rospy.loginfo('Foot placement failure : ' + str(e))
                return
            if paddle.buttons[2] and len(self.steps) < NUM_STEPS and not self._isPressing:
                self._isPressing = True
                rospy.loginfo('Footstep added: ' + PADDLE_NAMES[i])
                self.steps.append(feet[i])

        # Publish new footstep positions
        self.foot_markers.markers[0].pose = feet[0].pose
        self.foot_markers.markers[1].pose = feet[1].pose
        self.feet.publish(self.foot_markers)

        # Undo a step if some button is pressed
        for (i, paddle) in enumerate(msg.paddles):
            if paddle.buttons[4] and len(self.steps) > 0 and not self._isPressing:
                self._isPressing = True
                rospy.loginfo('Last footstep removed.')
                del self.steps[-1]

        # If we are happy with the footsteps then walk
        for (i, paddle) in enumerate(msg.paddles):
            if paddle.buttons[0] and len(self.steps) > 3 and not self._isPressing:
                self._isPressing = True
                rospy.loginfo('Walking!')
                self._isWalking = True
                return

        # RESET robot if necessary
        if msg.paddles[0].buttons[5] and not self._isPressing:
            self._isWalking = False
            self._isPressing = True
            rospy.loginfo('Harnessing robot...')
            self.reset_to_harnessed()
            rospy.loginfo('Harnessing complete.')
        elif msg.paddles[1].buttons[5] and not self._isPressing:
            self._isWalking = False
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
        self.control_mode.publish("Freeze")
        self.control_mode.publish("StandPrep")
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
    def compute_step(self, is_right_foot, trans, rot):
        
        # Create a new step object
        step = AtlasBehaviorStepData()

        # Bookkeeping for step and foot
        step.step_index = len(self.steps) + 1
        step.foot_index = is_right_foot
        step.duration = self.params["Stride Duration"]["value"]
        step.swing_height = self.params["Swing Height"]["value"]
            
        # Project foot position into XY plane
        step.pose.position.x = trans[0]
        step.pose.position.y = trans[1]
        step.pose.position.z = self.params["Step Height"]["value"]
         
        # Project rotation into XY plane
        (roll, pitch, yaw) = euler_from_quaternion(rot, axes='sxyz')
        Q = quaternion_from_euler(0, 0, yaw, axes='sxyz')

        step.pose.orientation.x = Q[0]
        step.pose.orientation.y = Q[1]
        step.pose.orientation.z = Q[2]
        step.pose.orientation.w = Q[3]

        # Return computed position
        return step
        
    # Send a trajectory of step commands when walking
    def process_atlas(self, state):
        if not self._isWalking:
            return

        walk_goal = AtlasSimInterfaceCommand()
        walk_goal.behavior = AtlasSimInterfaceCommand.WALK
        walk_goal.walk_params.use_demo_walk = False

        # 0 for full BDI control, 255 for PID control
        walk_goal.k_effort =  [0] * 28
               
        # Create a dummy step
        dummy_step = AtlasBehaviorStepData()
        dummy_step.step_index = 0
        dummy_step.foot_index = 1 - self.steps[0].foot_index

        # Load in the step array and pad it out
        steps = [ dummy_step ] + self.steps + [ dummy_step ] * 4

        # Check if we have finished walking
        if state.behavior_feedback.walk_feedback.current_step_index >= len(self.steps):

            # Clear step queue
            self.steps = []
        
            # Reset robot to standing
            stand_goal = AtlasSimInterfaceCommand()
            stand_goal = AtlasSimInterfaceCommand.STAND
            self.command.publish(stand_goal)
            self._isWalking = False

        else:
            # Send next steps to ATLAS
            next_idx = state.behavior_feedback.walk_feedback.next_step_index_needed
            walk_goal.walk_params.step_data = steps[next_idx : next_idx + 4]
            self.command.publish(walk_goal)

            
if __name__ == '__main__':
    rospy.init_node('walking_client')
    teleop = AtlasTeleop()
    teleop.run()
