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
from tf.transformations import quaternion_from_euler

import actionlib
import math
import rospy
import select
import sys
import termios
import tty

class AtlasTeleop():
    
    # BDI Controller bindings
    params = {"Forward Stride Length":{ "value":0.15, "min":0, "max":1, \
                                "type":"float"},
              "Lateral Stride Length":{ "value":0.15, "min":0, "max":1, \
                                "type":"float"},
              "Step Height":{"value":0, "min":-1, "max":1, "type":"float"},
              "Stride Duration":{ "value":0.63, "min": 0, "max":100, \
                                "type":"float"},
              "Walk Sequence Length":{"value":2, "min":2, "max":2, \
                                "type":"int"},
              "Stride Width":{"value":0.2, "min":0, "max":1, "type":"float"},
              "In Place Turn Size":{"value":math.pi / 16, "min":0, \
                                    "max":math.pi / 2, "type":"float"},
              "Turn Radius":{"value":2, "min":0.01, "max":100, "type":"float"},
              "Swing Height":{"value":0.3, "min":0, "max":1, "type":"float"}}
    
    def init(self):
        self._isResetting = False;
        self._isWalking = False;
        
        # Connects to necessary command topics
        self.command = rospy.Publisher('atlas/atlas_sim_interface_command', \
          AtlasSimInterfaceCommand)
        self.mode = rospy.Publisher('/atlas/mode', String, None, False, \
          True, None)
        self.control_mode = rospy.Publisher('/atlas/control_mode', \
          String, None, False, True, None)

        # Listen for hydra messages
        rospy.Subscriber("/legs/hydra_calib", Hydra, self.process_hydra)
    
    def run(self):
        self.init()
        rospy.spin()

    def process_hydra(self, msg):
        # Drive the hydra around
        if msg.paddles[0].buttons[6] or msg.paddles[1].buttons[6]:
            if not self._isWalking:
                self._isWalking = True
                self.twist(msg.paddles[0].joy[1], msg.paddles[0].joy[0], msg.paddles[1].joy[0])
        else:
            self._isWalking = False

        # RESET robot if necessary
        if msg.paddles[0].buttons[5] or msg.paddles[1].buttons[5]:
            if not self._isResetting:
                self._isResetting = True
                rospy.loginfo('Resetting robot...')
                self.reset_to_standing()
                rospy.loginfo('Resetting complete.')
        else:
            self._isResetting = False

    # Publishes commands to reset robot to a standing position
    def reset_to_standing(self):
        self.mode.publish("harnessed")
        self.control_mode.publish("Freeze")
        self.control_mode.publish("StandPrep")
        rospy.sleep(2.0)
        self.mode.publish("nominal")
        rospy.sleep(0.3)
        self.control_mode.publish("Stand")
        
    # Builds a trajectory of step commands. 
    # Param forward: 1 forward, -1 backward or 0 if no forward component
    # Param lateral: 1 left, -1 right, 0 if no lateral component
    # Param turn: 1 Counter clockwise turn, -1 clockwise turn    
    def twist(self, forward, lateral, turn):
        print str(forward) + ',' + str(lateral) + ',' +  str(turn)
        steps = []
        
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
        
        # This home step doesn't currently do anything, but it's a 
        # response to bdi not visiting the first step in a trajectory
        home_step = AtlasBehaviorStepData()
        
        # If moving right, first dummy step is on the left
        home_step.foot_index = 1*(lateral < 0)
        home_step.pose.position.y = 0.1
        steps.append(home_step)
        prevX = 0
        prevY = 0
        
        # Builds the sequence of steps needed
        for i in range(self.params["Walk Sequence Length"]["value"]):
            theta += (turn != 0) * dTheta
            
            # is_right_foot = 1, when stepping with right
            is_even = i%2
            is_odd = 1 - is_even
            is_right_foot = is_even
            is_left_foot = is_odd

            # left = 1, right = -1            
            foot = 1 - 2 * is_right_foot
            
            if turn == 0:
                X = (forward != 0) * (X + forward * L)
                Y = (lateral != 0) * (Y + is_odd * lateral * L_lat) + foot * W / 2
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
             
            Q = quaternion_from_euler(0, 0, theta)
            step = AtlasBehaviorStepData()
            
            # One step already exists, so add one to index
            step.step_index = i+1
            
            # Alternate between feet, start with left
            step.foot_index = is_right_foot
            
            #If moving laterally to the left, start with the right foot
            if (lateral > 0):
                step.foot_index = is_left_foot
            
            step.duration = self.params["Stride Duration"]["value"]
            
            step.pose.position.x = X
            step.pose.position.y = Y
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
            Y = Y - foot * W
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
            
        Q = quaternion_from_euler(0, 0, theta)
        step = AtlasBehaviorStepData()
        step.step_index = len(steps)
        step.foot_index = is_right_foot
        step.duration = self.params["Stride Duration"]["value"]
        step.pose.position.x = X
        step.pose.position.y = Y
        step.pose.position.z = self.params["Step Height"]["value"]
        step.pose.orientation.x = Q[0]
        step.pose.orientation.y = Q[1]
        step.pose.orientation.z = Q[2]
        step.pose.orientation.w = Q[3]
        step.swing_height = self.params["Swing Height"]["value"]
        
        steps.append(step)

        # 0 for full BDI control, 255 for PID control
        k_effort =  [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
               
        walk_goal = AtlasSimInterfaceCommand()
        walk_goal.behavior = AtlasSimInterfaceCommand.WALK
        walk_goal.walk_params.step_data = steps
        walk_goal.walk_params.use_demo_walk = False
        print(str(walk_goal))
        
        self.command.publish(walk_goal)
        for step in steps:
            rospy.logdebug("foot: " + str(step.foot_index) + \
              " [" + str(step.pose.position.x) + \
              ", " + str(step.pose.position.y) + ", " + str(theta) + "]")   
            
if __name__ == '__main__':
    rospy.init_node('walking_client')
    teleop = AtlasTeleop()
    teleop.run()
