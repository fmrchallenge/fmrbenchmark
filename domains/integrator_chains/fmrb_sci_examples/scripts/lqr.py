#!/usr/bin/env python
from __future__ import print_function

import roslib; roslib.load_manifest('fmrb_sci_examples')
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
        self.intopic.publish(VectorStamped(v=Vector(-np.dot(self.K, np.asarray(vs.v.point)))))

class LQRController(StateFeedback):
    def __init__(self, intopic, outtopic,
                 A, B, Q=None, R=None):
        if Q is None:
            Q = np.eye(A.shape[0])
        if R is None:
            R = np.eye(B.shape[1])
        K, S, E = lqr(A,B,Q,R)
        StateFeedback.__init__(self, intopic, outtopic, K)


if __name__ == "__main__":
    rospy.init_node("lqr", anonymous=True)

    n = rospy.get_param("dynamaestro/output_dim")
    m = rospy.get_param("dynamaestro/number_integrators")

    A = np.diag(np.ones((m-1)*n), k=n)
    B = np.zeros((m*n, n))
    B[(m-1)*n:,:] = np.eye(n)
    Q = np.eye(m*n)
    R = np.eye(n)

    lqrc = LQRController("input", "output", A, B, Q, R)
    rospy.spin()
