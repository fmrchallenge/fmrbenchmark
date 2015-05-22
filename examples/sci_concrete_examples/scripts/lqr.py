#!/usr/bin/env python
from __future__ import print_function
import time
import numpy as np
import numpy.linalg as la
from control import lqr

import roslib; roslib.load_manifest('sci_concrete_examples')
import rospy
from dynamaestro.msg import VectorStamped, Vector
from dynamaestro.srv import DMMode, DMModeRequest

from fmrb import integrator_chains


class StateFeedback:
    def __init__(self, intopic, outtopic, K=None):
        self.error = None
        self.intopic = rospy.Publisher(intopic, VectorStamped, queue_size=1)
        self.outtopic = rospy.Subscriber(outtopic, VectorStamped, self.read_state)
        self.K = K

    def read_state(self, vs):
        self.error = np.asarray(vs.v.point) - self.target_state
        self.intopic.publish(VectorStamped(v=Vector(-np.dot(self.K, self.error))))

    def unregister(self):
        self.outtopic.unregister()
        self.intopic.unregister()

class LQRController(StateFeedback):
    def __init__(self, intopic, outtopic,
                 A, B, target=None, Q=None, R=None):
        if Q is None:
            Q = np.eye(A.shape[0])
        if R is None:
            R = np.eye(B.shape[1])
        self.target_state = np.zeros(A.shape[0])
        if target is not None:
            self.target_state[:target.shape[0]] = target
        K, S, E = lqr(A,B,Q,R)
        StateFeedback.__init__(self, intopic, outtopic, K)


if __name__ == "__main__":
    rospy.init_node("lqr", anonymous=True)

    dmmode = rospy.ServiceProxy("dynamaestro/mode", DMMode)
    while not dmmode(DMModeRequest.READY):
        time.sleep(0.5)
    assert dmmode(DMModeRequest.START)

    while not rospy.is_shutdown():
        probstr = rospy.get_param("dynamaestro/probleminstance", None)
        if probstr is not None:
            break

    prob = integrator_chains.Problem.loadJSON(probstr)
    n = prob.output_dim
    m = prob.number_integrators

    A = np.diag(np.ones((m-1)*n), k=n)
    B = np.zeros((m*n, n))
    B[(m-1)*n:,:] = np.eye(n)
    Q = np.eye(m*n)
    R = np.eye(n)

    targets = []
    for goal in prob.goals:
        target_polytopeV = goal.getVrep()
        targets.append( np.mean(target_polytopeV, axis=0) );

    current = 0
    while not rospy.is_shutdown():
        lqrc = LQRController("input", "output", A, B, targets[current], Q, R)
        while (not rospy.is_shutdown()
               and ((lqrc.error is None) or (la.norm(lqrc.error) > 0.01))):
            time.sleep(0.5)
        print('Reached '+prob.goals[current].label)
        lqrc.unregister()
        current += 1
        if current >= len(prob.goals):
            current = 0
