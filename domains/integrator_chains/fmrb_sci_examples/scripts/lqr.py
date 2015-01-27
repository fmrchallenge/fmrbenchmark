#!/usr/bin/env python
from __future__ import print_function

import roslib; roslib.load_manifest('dynamaestro')
import rospy
from dynamaestro.msg import VectorStamped

from control import lqr
import numpy as np


class LQRController(rospy.Subscriber):
    def __init__(self, intopic, outtopic):
        rospy.Subscriber.__init__(self, outtopic, VectorStamped, self.read_state)
        self.intopic = rospy.Publisher(intopic, VectorStamped, queue_size=1)
        A = np.array([[0., 1, 0], [0,0,1], [0,0,0]])
        B = np.array([[0.],[0],[1]])
        Q = np.diag([1.,1,1])
        R = np.diag([1.])
        K, S, E = lqr(A,B,Q,R)
        self.K = K

    def read_state(self, vs):
        self.intopic.publish(VectorStamped(point=[-(vs.point[0] + 2.4142*vs.point[1] + 2.4142*vs.point[2])]))


if __name__ == "__main__":
    rospy.init_node("lqr", anonymous=True)
    lqrc = LQRController("input", "output")
    rospy.spin()
