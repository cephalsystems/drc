
#!/usr/bin/env python
#
# Read from a TF frame and attempt to create an IK setpoint that
# puts the given end effector at that TF frame.
#

import roslib; roslib.load_manifest('kdl_hydra')
import rospy

from tf import TransformListener
from pykdl_utils import kdl_parser
from urdf_parser_py.urdf import URDF
import PyKDL as kdl
from numpy import zeros, diag

from atlas_msgs.msg import AtlasState
from atlas_msgs.msg import AtlasCommand

import threading

# TODO: automatically construct this from AtlasState message
ATLAS_JOINT_NAMES = [ 'back_lbz', 'back_mby', 'back_ubx', 'neck_ay',
                      'l_leg_uhz', 'l_leg_mhx', 'l_leg_lhy', 'l_leg_kny', 'l_leg_uay', 'l_leg_lax',
                      'r_leg_uhz', 'r_leg_mhx', 'r_leg_lhy', 'r_leg_kny', 'r_leg_uay', 'r_leg_lax',
                      'l_arm_usy', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx',
                      'r_arm_usy', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx']

class TeleopLimb:
    _q_current = None
    _q_desired_vel = None

    _fk = None
    _ik = None
    _ikv = None
    
    _tf = None

    _tf_root = None
    _kdl_chain = None
    _urdf_chain = None
    _name_chain = None
    _idx_chain = None

    _kp = None
    _ki = None
    _kd = None
    _i_clamp = None

    _num_joints = None

    _lock = threading.Lock()

    def __init__(self, tf, robot_urdf, robot_kdl, start_link, end_link):

        # Store transform filter and root frame for later
        self._tf = tf
        self._tf_root = '/' + start_link

        # Define manipulator chain from KDL tree
        self._kdl_chain = robot_kdl.getChain(start_link, end_link)
        self._name_chain = [ self._kdl_chain.getSegment(idx).getJoint().getName()
                             for idx in xrange(0, self._kdl_chain.getNrOfJoints()) ]
        self._idx_chain = [ ATLAS_JOINT_NAMES.index(name) for name in self._name_chain ]
        self._urdf_chain = [ robot_urdf.joints[name] for name in self._name_chain ]
        self._num_joints = len(self._urdf_chain)

        self._q_current = kdl.JntArray(self._num_joints)
        self._q_desired_vel = kdl.JntArray(self._num_joints)
        
        # Define the joint limits from the URDF 
        joint_min = kdl.JntArray(self._num_joints)
        joint_max = kdl.JntArray(self._num_joints)
        for idx in xrange(0, self._num_joints):
            joint_min[idx] = self._urdf_chain[idx].limits.lower
            joint_max[idx] = self._urdf_chain[idx].limits.upper

        # Initialize IK for this manipulator
        self._fk = kdl.ChainFkSolverPos_recursive(self._kdl_chain)
        self._ikv = kdl.ChainIkSolverVel_pinv(self._kdl_chain)
        #self._ikv = kdl.ChainIkSolverVel_wdls(self._kdl_chain)
        #self._ikv.setWeightTS(diag([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]).tolist())
        self._ik = kdl.ChainIkSolverPos_NR_JL(self._kdl_chain,
                                         joint_min, joint_max,
                                         self._fk, self._ikv,
                                         1000, 1e-3)

        # Load each gain into dictionary
        self._kp = [ rospy.get_param('atlas_controller/gains/' + name + '/p')
                     for name in self._name_chain ] 
        self._ki = [ rospy.get_param('atlas_controller/gains/' + name + '/i') 
                     for name in self._name_chain ] 
        self._kd = [ rospy.get_param('atlas_controller/gains/' + name + '/d') 
                     for name in self._name_chain ] 
        self._i_clamp = [ rospy.get_param('atlas_controller/gains/' + name + '/i_clamp') 
                          for name in self._name_chain ]

    def update(self, atlas_state):
        with self._lock:
            # Initialize current configuration when we get our first message
            if not self._q_current:
                self._q_current = kdl.JntArray(self._num_joints)
                
            # Populate current configuration from state message
            for idx in xrange(0, self._num_joints):
                self._q_current[idx] = atlas_state.position[self._idx_chain[idx]]


    def solve(self, tf_target):
        with self._lock:
            # Compute the relative transform to the end link
            try:
                pos, quat = self._tf.lookupTransform(self._tf_root, tf_target, rospy.Time(0))
            except Exception as e:
                raise Exception("Transform failed from {0} {1}".format(self._tf_root, tf_target))

            x_desired = kdl.Frame(kdl.Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3]),
                                  kdl.Vector(pos[0], pos[1], pos[2]))
                
            # Compute twist to desired end point
            x_actual = kdl.Frame()
            self._fk.JntToCart(self._q_current, x_actual)
            x_vel = kdl.Twist((x_desired.p - x_actual.p), kdl.Vector())

            # Compute IK for desired joint positions
            q_desired_vel = kdl.JntArray(self._q_desired_vel)
            success = self._ikv.CartToJnt(self._q_current, x_vel, q_desired_vel)
            if (success >= 0):
                self._q_desired_vel = q_desired_vel

                q_max = max(abs(q_desired_vel[i]) for i in xrange(0, self._num_joints))
                for i in xrange(0, self._num_joints):
                    self._q_desired_vel[i] /= q_max

            else:
                raise Exception('IK failed [{0}]'.format(success))

    def populate(self, atlas_command):
        with self._lock:
            # Fill in relevant components of command to atlas
            for idx in xrange(0, self._num_joints):
                atlas_command.k_effort[self._idx_chain[idx]] = 255.0

                # Servo using velocity
                atlas_command.velocity[self._idx_chain[idx]] = 6.0 * self._q_desired_vel[idx]
                atlas_command.kp_velocity[self._idx_chain[idx]] = 10.0

                # Zero out position component
                atlas_command.kp_position[self._idx_chain[idx]] = 0.0
                atlas_command.ki_position[self._idx_chain[idx]] = 0.0
                atlas_command.kd_position[self._idx_chain[idx]] = 0.0


    def clear(self, atlas_command):
        with self._lock:
            # Fill in relevant components of command to atlas
            for idx in xrange(0, self._num_joints):

                # Zero out velocity component
                atlas_command.kp_velocity[self._idx_chain[idx]] = 0.0
                atlas_command.k_effort[self._idx_chain[idx]] = 255.0

                # Hold at current position
                atlas_command.position[self._idx_chain[idx]] = self._q_current[idx] or 0.0
                atlas_command.kp_position[self._idx_chain[idx]] = self._kp[idx]
                atlas_command.ki_position[self._idx_chain[idx]] = self._ki[idx]
                atlas_command.kd_position[self._idx_chain[idx]] = self._kd[idx]


