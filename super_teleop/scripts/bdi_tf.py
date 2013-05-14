#! /usr/bin/env python
import roslib; roslib.load_manifest('super_teleop')

from atlas_msgs.msg import AtlasSimInterfaceState
from sensor_msgs.msg import Imu

import tf
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

import math
import rospy

class BdiTf():
    
    state = None
    imu = None

    def init(self):
        # Connects to necessary topics
        rospy.Subscriber('atlas/atlas_sim_interface_state', 
                         AtlasSimInterfaceState, self.atlas_callback)
        rospy.Subscriber('/atlas/imu', Imu, self.imu_callback)


    def run(self):
        self.init()
        br = tf.TransformBroadcaster()

        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.state and self.imu:
                br.sendTransform([ self.state.pos_est.position.x,
                                   self.state.pos_est.position.y,
                                   self.state.pos_est.position.z ],
                                 [ self.imu.orientation.x,
                                   self.imu.orientation.y,
                                   self.imu.orientation.z,
                                   self.imu.orientation.w ],
                                 rospy.Time.now(),
                                 "pelvis",
                                 "world")
            r.sleep()

    def atlas_callback(self, msg):
        self.state = msg

    def imu_callback(self, msg):
        self.imu = msg

if __name__ == '__main__':
    rospy.init_node('bdi_tf')
    bdi_tf = BdiTf()
    bdi_tf.run()
