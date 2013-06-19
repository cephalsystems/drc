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
from tf.transformations import quaternion_from_euler, \
                               euler_from_quaternion

def grouper(n, iterable, fillvalue=None):
    "grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"
    args = [iter(iterable)] * n
    return itertools.izip_longest(*args, fillvalue=fillvalue)

def signum(int):
    if(int <= 0): return -1;
    elif(int > 0): return 1;
    else: return int;

class AtlasTeleop(object):

    # Keyboard teleop bindings
    keys = { 
        'u': {"steps":4, "radius": 1.4 }, 
        'i': {"steps":4, "radius": 100 }, 
        'o': {"steps":4, "radius":-1.4 }, 
        'j': {"steps":4, "radius": 0.1 }, 
        'l': {"steps":4, "radius":-0.1 }
        }
    
    # BDI Controller bindings 
    params = {"Forward Stride Length":{ "value":0.15, "min":0, "max":1, \
                                "type":"float"},
              "Stride Length Interval":{"value":0.05, "min":0, "max":1, \
                                "type":"float"},
              "Step Height":{"value":0, "min":-1, "max":1, "type":"float"}, \
              "Stride Duration":{ "value":0.63, "min": 0, "max":100, \
                                "type":"float"},
              "Stride Width":{"value":0.25, "min":0, "max":1, "type":"float"},
              "In Place Turn Size":{"value":math.pi / 16, "min":0, \
                                    "max":math.pi / 2, "type":"float"},
              "Swing Height":{"value":0.3, "min":0, "max":1, "type":"float"}}

    steps = []

    
    def init(self):
        self._isWalking = False;
        
        # Connects to necessary command topics
        self.command = rospy.Publisher('/atlas/atlas_sim_interface_command', \
                                       AtlasSimInterfaceCommand)
        self.service = rospy.Service('walk', Walk, self.walk_service)
        self.listen = rospy.Subscriber("/atlas/atlas_sim_interface_state", \
                                           AtlasSimInterfaceState, \
                                           self.process_atlas, queue_size=1)
        
    def run(self):
        self.init()
        rospy.loginfo('Starting up leg teleop...')
        rospy.spin()
        rospy.loginfo('Shutting down leg teleop...')

    def walk_service(self, req):
        self._isWalking = False;
        self.steps = []
        self.load_position()

        for command in req.commands:            
            if self.keys.has_key(command):
                cmd = self.keys[command]                
                for i in range(cmd["steps"]):
                    self.add_step(cmd["radius"])

        if (len(self.steps) > 0):
            self._isWalking = True;
            rospy.loginfo('Started walk with {0} steps'.format(len(self.steps)))
            while(self._isWalking):
                pass
            rospy.sleep(1.0)
            rospy.loginfo('Completed walk with {0} steps'.format(len(self.steps)))

        return Empty

    def load_position(self):
        self.steps = []
        
        step = AtlasBehaviorStepData()
        step.step_index = self.state.walk_feedback.current_step_index
        step.pose = self.state.foot_pos_est[0]

        self.steps.append(step)


    def add_step(self, radius):
        L = self.params["Forward Stride Length"]["value"]
        W = self.params["Stride Width"]["value"]
        T = self.params["In Place Turn Size"]["value"]
        R = radius

        # Get origin based on last step
        if not self.steps:
            raise Exception("Unknown foot positions.")

        last_step = self.steps[-1]
        rpy = euler_from_quaternion([
                last_step.pose.orientation.x,
                last_step.pose.orientation.y,
                last_step.pose.orientation.z,
                last_step.pose.orientation.w,
                ])
        
        Th = rpy[2]
        X = last_step.pose.position.x + signum(last_step.foot_index) * W/2 * math.sin(Th)
        Y = last_step.pose.position.y - signum(last_step.foot_index) * W/2 * math.cos(Th)
        
        # Transform from here to next step center
        if (R > L):
            dTh = math.asin(L / R)
        else:
            dTh = signum(R) * T

        dX = R * math.sin(dTh)
        dY = R * (1 - math.cos(dTh))
        
        # Transform to next foot position
        dX = dX + signum(last_step.foot_index) * W/2 * math.sin(dTh)
        dY = dY - signum(last_step.foot_index) * W/2 * math.cos(dTh)

        # Store this foot position
        Q = quaternion_from_euler(0, 0, Th + dTh)

        step = AtlasBehaviorStepData()
        step.step_index = last_step.step_index + 1
        step.foot_index = 1 if (last_step.foot_index == 0) else 0
        step.duration = self.params["Stride Duration"]["value"]
            
        step.pose.position.x = X + math.cos(Th)*dX - math.sin(Th)*dY
        step.pose.position.y = Y + math.sin(Th)*dX + math.cos(Th)*dY
        step.pose.position.z = self.params["Step Height"]["value"]
         
        step.pose.orientation.x = Q[0]
        step.pose.orientation.y = Q[1]
        step.pose.orientation.z = Q[2]
        step.pose.orientation.w = Q[3]
            
        step.swing_height = self.params["Swing Height"]["value"]         
        self.steps.append(step)

    def final_step(self):
        pass
        
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
               
        # Load in the step array and pad it out
        steps = self.steps + [ self.steps[-1] ] * 5

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
