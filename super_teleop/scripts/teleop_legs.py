#! /usr/bin/env python
import roslib; roslib.load_manifest('super_teleop')

from atlas_msgs.msg import AtlasSimInterfaceCommand, \
                           AtlasSimInterfaceState, \
                           AtlasBehaviorStepData, \
                           AtlasBehaviorWalkParams, \
                           AtlasBehaviorStandParams
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Empty
from super_teleop.srv import Walk

import sys
import math
import rospy
import itertools
from tf.transformations import quaternion_from_euler

def grouper(n, iterable, fillvalue=None):
    "grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"
    args = [iter(iterable)] * n
    return itertools.izip_longest(*args, fillvalue=fillvalue)

class AtlasTeleop(object):

    # Keyboard teleop bindings
    dynamic_dir = {'u': {"forward":1, "lateral":0, "turn": 1}, \
                  'i': {"forward":1, "lateral":0, "turn": 0}, \
                  'o': {"forward":1, "lateral":0, "turn":-1}, \
                  'j': {"forward":0, "lateral":1, "turn": 0}, \
                  'k': {"forward":0, "lateral":0, "turn": 0}, \
                  'l': {"forward":0, "lateral":-1, "turn": 0}, \
                  'm': {"forward":0, "lateral":0, "turn": 0.5}, \
                  ',': {"forward":-0.5, "lateral":0, "turn": 0}, \
                  '.': {"forward":0, "lateral":0, "turn":-0.5}}

    static_dir = {'U': {"forward":1, "lateral":0, "turn": 1}, \
                  'I': {"forward":1, "lateral":0, "turn": 0}, \
                  'O': {"forward":1, "lateral":0, "turn":-1}, \
                  'J': {"forward":0, "lateral":1, "turn": 0}, \
                  'K': {"forward":-1, "lateral":0, "turn": 0}, \
                  'L': {"forward":0, "lateral":-1, "turn": 0}, \
                  'M': {"forward":0, "lateral":0, "turn": 0.5}, \
                  '<': {"forward":-0.5, "lateral":0, "turn": 0}, \
                  '>': {"forward":0, "lateral":0, "turn":-0.5}}
    
    # BDI Controller bindings 
    params = {"Forward Stride Length":{ "value":0.15, "min":0, "max":1, \
                                "type":"float"},
              "Stride Length Interval":{"value":0.05, "min":0, "max":1, \
                                "type":"float"},
              "Lateral Stride Length":{ "value":0.15, "min":0, "max":1, \
                                "type":"float"},
              "Step Height":{"value":0, "min":-1, "max":1, "type":"float"}, \
              "Stride Duration":{ "value":0.63, "min": 0, "max":100, \
                                "type":"float"},
              "Walk Sequence Length":{"value":5, "min":1, "max":sys.maxint, \
                                "type":"int"},
              "Stride Width":{"value":0.2, "min":0, "max":1, "type":"float"},
              "In Place Turn Size":{"value":math.pi / 16, "min":0, \
                                    "max":math.pi / 2, "type":"float"},
              "Turn Radius":{"value":2, "min":0.01, "max":100, "type":"float"},
              "Swing Height":{"value":0.3, "min":0, "max":1, "type":"float"}}

    steps = []

    
    def init(self):
        self._isWalking = False;
        
        # Connects to necessary command topics
        self.command = rospy.Publisher('/atlas/atlas_sim_interface_command', \
                                       AtlasSimInterfaceCommand)
        self.mode = rospy.Publisher('/atlas/mode', String, None, False, \
                                    True, None)
        self.control_mode = rospy.Publisher('/atlas/control_mode', \
                                            String, None, False, True, None)
        self.service = rospy.Service('walk', Walk, self.walk_service)
        self.listen = rospy.Subscriber("/atlas/atlas_sim_interface_state", \
                                           AtlasSimInterfaceState, \
                                           self.process_atlas, queue_size=1)
        
    def run(self):
        self.init()
        rospy.loginfo('Starting up leg teleop...')
        rospy.spin()
        rospy.loginfo('Shutting down leg teleop...')

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

    def walk_service(self, req):
        for command in req.commands:
            self._isWalking = False;
            self.steps = []
            self.load_position()

            if self.dynamic_dir.has_key(command):
                dir = self.dynamic_dir[command]
                self.steps += self.build_steps(dir["forward"], dir["lateral"], dir["turn"], False)
            elif self.static_dir.has_key(command):
                dir = self.dynamic_dir[command]
                self.steps += self.build_steps(dir["forward"], dir["lateral"], dir["turn"], True)

            self._isWalking = True;
            rospy.loginfo('Started walk with {0} steps'.format(len(self.steps)))
            while(self._isWalking):
                pass
            rospy.sleep(0.5)
            rospy.loginfo('Completed walk with {0} steps'.format(len(self.steps)))

        return Empty

    def load_position(self):
        self.X = self.state.pos_est.position.x
        self.Y = self.state.pos_est.position.y
        self.theta = math.atan2(self.state.foot_pos_est[1].position.x - 
                                self.state.foot_pos_est[0].position.x,
                                self.state.foot_pos_est[0].position.y - 
                                self.state.foot_pos_est[1].position.y)

    def build_steps(self, forward, lateral, turn, is_static):
        L = self.params["Forward Stride Length"]["value"]
        L_lat = self.params["Lateral Stride Length"]["value"]
        R = self.params["Turn Radius"]["value"]
        W = self.params["Stride Width"]["value"]
        X = 0
        Y = 0
        theta = 0
        dTheta = 0
        
        if forward != 0:
            dTheta = turn * 2 * math.asin(L / (2 * (R + \
            self.params["Stride Width"]["value"]/2)))
        else:
            dTheta = turn * self.params["In Place Turn Size"]["value"]
        steps = []

        
        if forward != 0:
            dTheta = turn * 2 * math.asin(L / (2 * (R + \
            self.params["Stride Width"]["value"]/2)))
        else:
            dTheta = turn * self.params["In Place Turn Size"]["value"]
        steps = []
        
        # This home step doesn't currently do anything, but it's a 
        # response to bdi not visiting the first step in a trajectory
        # home_step = AtlasBehaviorStepData()
        
        # If moving right, first dummy step is on the left
        # home_step.foot_index = 1*(lateral < 0)
        # home_step.pose.position.y = 0.1
        # steps.append(home_step)
        prevX = 0
        prevY = 0
        
        # Builds the sequence of steps needed
        for i in range(self.params["Walk Sequence Length"]["value"]):
            # is_right_foot = 1, when stepping with right
            is_even = i%2
            is_odd = 1 - is_even
            is_right_foot = is_even
            is_left_foot = is_odd

            # left = 1, right = -1            
            foot = 1 - 2 * is_right_foot
            
            if is_static:
                theta = (turn != 0) * dTheta
                if turn == 0:
                    X = (forward != 0) * (forward * L)
                    Y = (lateral != 0) * (is_odd * lateral * L_lat) + \
                        foot * W / 2
                elif forward != 0:
                    # Radius from point to foot (if turning)
                    R_foot = R + foot * W/2
                    
                    # turn > 0 for CCW, turn < 0 for CW
                    X = forward * turn * R_foot * math.sin(theta)
                    Y = forward * turn * (R - R_foot*math.cos(theta))
                    
                    rospy.logdebug("R: " + str(R) + " R_foot:" + \
                    str(R_foot) + " theta: " + str(theta) +  \
                    " math.sin(theta): " + str(math.sin(theta)) + \
                    " math.cos(theta) + " + str(math.cos(theta)))
                elif turn != 0:
                    X = turn * W/2 * math.sin(theta)
                    Y = turn * W/2 * math.cos(theta)
            else:
                theta += (turn != 0) * dTheta
                if turn == 0:
                    X = (forward != 0) * (X + forward * L)
                    Y = (lateral != 0) * (Y + is_odd * lateral * L_lat) + \
                        foot * W / 2
                elif forward != 0:
                    # Radius from point to foot (if turning)
                    R_foot = R + foot * W/2
                    
                    # turn > 0 for CCW, turn < 0 for CW
                    X = forward * turn * R_foot * math.sin(theta)
                    Y = forward * turn * (R - R_foot*math.cos(theta))
                    
                    rospy.logdebug("R: " + str(R) + " R_foot:" + \
                    str(R_foot) + " theta: " + str(theta) +  \
                    " math.sin(theta): " + str(math.sin(theta)) + \
                    " math.cos(theta) + " + str(math.cos(theta)))
                elif turn != 0:
                    X = turn * W/2 * math.sin(theta)
                    Y = turn * W/2 * math.cos(theta)
                    
             
            Q = quaternion_from_euler(0, 0, theta + self.theta)
            step = AtlasBehaviorStepData()
            
            # One step already exists, so add one to index
            step.step_index = i
            
            # Alternate between feet, start with left
            step.foot_index = is_right_foot
            
            #If moving laterally to the left, start with the right foot
            if (lateral > 0):
                step.foot_index = is_left_foot
            
            step.duration = self.params["Stride Duration"]["value"]
            
            step.pose.position.x = self.X + math.cos(self.theta)*X - math.sin(self.theta)*Y
            step.pose.position.y = self.Y + math.sin(self.theta)*X + math.cos(self.theta)*Y
            step.pose.position.z = self.params["Step Height"]["value"]
         
            step.pose.orientation.x = Q[0]
            step.pose.orientation.y = Q[1]
            step.pose.orientation.z = Q[2]
            step.pose.orientation.w = Q[3]
            
            step.swing_height = self.params["Swing Height"]["value"]         
            steps.append(step)
        
        # Add final step to bring feet together
        is_right_foot = 1 - steps[-1].foot_index
        is_even = is_right_foot
        # foot = 1 for left, foot = -1 for right
        foot = 1 - 2 * is_right_foot
        
        if turn == 0:
            Y = Y + foot * W
        elif forward != 0:
            rospy.logdebug("R: " + str(R) + " R_foot:" + \
            str(R_foot) + " theta: " + str(theta) +  \
           " math.sin(theta): " + str(math.sin(theta)) + \
           " math.cos(theta) + " + str(math.cos(theta)))
            
            # R_foot is radius to foot
            R_foot = R + foot * W/2
            #turn > 0 for counter clockwise
            X = forward * turn * R_foot * math.sin(theta)
            Y = forward * turn * (R - R_foot*math.cos(theta))
        else:
            X = turn * W/2 * math.sin(theta)
            Y = turn * W/2 * math.cos(theta)
            
        Q = quaternion_from_euler(0, 0, theta + self.theta)
        step = AtlasBehaviorStepData()
        step.step_index = len(steps)
        step.foot_index = is_right_foot
        step.duration = self.params["Stride Duration"]["value"]
        step.pose.position.x = self.X + math.cos(self.theta)*X - math.sin(self.theta)*Y
        step.pose.position.y = self.Y + math.sin(self.theta)*X + math.cos(self.theta)*Y
        step.pose.position.z = self.params["Step Height"]["value"]
        step.pose.orientation.x = Q[0]
        step.pose.orientation.y = Q[1]
        step.pose.orientation.z = Q[2]
        step.pose.orientation.w = Q[3]
        step.swing_height = self.params["Swing Height"]["value"]
        
        steps.append(step)
        return steps
        
    # Send a trajectory of step commands when walking
    def process_atlas(self, state):
        self.state = state;

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
        steps = self.steps + [ dummy_step ] * 5

        # Check if we have finished walking
        if state.walk_feedback.current_step_index >= len(self.steps) - 2:

            # Reset robot to standing
            stand_goal = AtlasSimInterfaceCommand()
            stand_goal.behavior = AtlasSimInterfaceCommand.STAND
            self.command.publish(stand_goal)

            # Clear step queue
            self.steps = []
            self._isWalking = False
            rospy.loginfo('Completed walk!')

        else:
            # Send next steps to ATLAS
            next_idx = state.walk_feedback.next_step_index_needed
            walk_goal.walk_params.step_queue = steps[next_idx : next_idx + 4]
            self.command.publish(walk_goal)

            
if __name__ == '__main__':
    rospy.init_node('walking_client')
    teleop = AtlasTeleop()
    teleop.run()
