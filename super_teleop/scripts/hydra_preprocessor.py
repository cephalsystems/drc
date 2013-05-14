#! /usr/bin/env python
import roslib; roslib.load_manifest('super_teleop')

from razer_hydra.msg import Hydra
import tf
import rospy

PADDLE_NAMES = [ 'hydra_left', 'hydra_right' ]

class HydraPreprocessor():

    def init(self):
        rospy.Subscriber("hydra_calib", Hydra, self.process_hydra)
        self.br = tf.TransformBroadcaster()
    
    def run(self):
        self.init()
        rospy.spin()

    def process_hydra(self, msg):
        for (i, paddle) in enumerate(msg.paddles):
            self.br.sendTransform([ paddle.transform.translation.x * 2.0,
                                    paddle.transform.translation.y * 2.0,
                                    paddle.transform.translation.z * 2.0 ],
                                  [ paddle.transform.rotation.x,
                                    paddle.transform.rotation.y,
                                    paddle.transform.rotation.z,
                                    paddle.transform.rotation.w ],
                                  rospy.Time.now(),
                                  PADDLE_NAMES[i],
                                  'hydra_base')


if __name__ == '__main__':
    rospy.init_node('hydra_preprocessor')
    hydra_pp = HydraPreprocessor()
    hydra_pp.run()

