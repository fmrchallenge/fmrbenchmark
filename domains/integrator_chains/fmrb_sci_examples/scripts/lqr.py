#!/usr/bin/env python
from __future__ import print_function

import roslib; roslib.load_manifest('dynamaestro')
import rospy
from dynamaestro.msg import VectorStamped, Vector

from control import lqr
import numpy as np


class StateFeedback(rospy.Subscriber):
    def __init__(self, intopic, outtopic, K=None):
        rospy.Subscriber.__init__(self, outtopic, VectorStamped, self.read_state)
        self.intopic = rospy.Publisher(intopic, VectorStamped, queue_size=1)
        self.K = K

    def read_state(self, vs):
        self.intopic.publish(VectorStamped(v=Vector(point=[-np.dot(self.K, np.asarray(vs.v.point))])))

class LQRController(StateFeedback):
    def __init__(self, intopic, outtopic,
                 A=None, B=None, Q=None, R=None):
        if A is None and B is None:
            A = np.array([[0., 1, 0],
                          [0,  0, 1],
                          [0,  0, 0]])
            B = np.array([[0.], [0], [1]])
        if Q is None and R is None:
            Q = np.diag([1.,1,1])
            R = np.diag([1.])
        K, S, E = lqr(A,B,Q,R)
        StateFeedback.__init__(self, intopic, outtopic, K)


if __name__ == "__main__":
    rospy.init_node("lqr", anonymous=True)

    n = 1  # Number of output dimensions

    # Number of derivatives
    m = rospy.get_param("number_integrators")

    A = np.diag(np.ones(m-1), k=1)
    B = np.zeros((m, 1))
    B[-1,0] = 1.0
    Q = np.diag(np.ones(m))
    R = np.diag([1.])

    lqrc = LQRController("input", "output", A, B, Q, R)
    rospy.spin()
