#!/usr/bin/env python
from __future__ import print_function
import time
import numpy as np
import numpy.linalg as la
from control import lqr

import roslib; roslib.load_manifest('sci_concrete_examples')
import rospy
from std_msgs.msg import Header
from dynamaestro.msg import VectorStamped, Vector, ProblemInstanceJSON
from dynamaestro.srv import DMMode, DMModeRequest

from fmrb import integrator_chains


class StateFeedback:
    def __init__(self, intopic, outtopic, K=None):
        self.error = None
        self.intopic = rospy.Publisher(intopic, VectorStamped, queue_size=1)
        while self.intopic.get_num_connections() == 0 and not rospy.is_shutdown():
            time.sleep(0.5)
        self.outtopic = rospy.Subscriber(outtopic, VectorStamped, self.read_state)
        self.K = K
        self.trial_ended = False

    def read_state(self, vs):
        if len(vs.v.point) == 0:
            self.trial_ended = True
            return
        self.error = np.asarray(vs.v.point) - self.target_state
        self.intopic.publish(VectorStamped(header=Header(stamp=rospy.Time.now()),
                                           v=Vector(-np.dot(self.K, self.error))))

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

class InstanceMonitor:
    def __init__(self, problemJSON_topic='dynamaestro/probleminstance_JSON'):
        self.instanceJSONsub = rospy.Subscriber(problemJSON_topic,
                                                ProblemInstanceJSON,
                                                self.get_newinstance, queue_size=1)
        self.last_updated = None
        self.prob = None
        self.busy = False

    def get_newinstance(self, pinst):
        self.busy = True
        self.prob = integrator_chains.Problem.loadJSON(pinst.problemjson)
        self.last_updated = pinst.stamp
        self.busy = False


def main(imon):
    start_time = rospy.Time.now()
    dmmode = rospy.ServiceProxy("dynamaestro/mode", DMMode)
    while not dmmode(DMModeRequest.READY):
        time.sleep(0.5)
    assert dmmode(DMModeRequest.START)

    while (not rospy.is_shutdown()
           and (imon.last_updated is None or start_time > imon.last_updated)):
        time.sleep(0.5)

    n = imon.prob.output_dim
    m = imon.prob.number_integrators

    A = np.diag(np.ones((m-1)*n), k=n)
    B = np.zeros((m*n, n))
    B[(m-1)*n:,:] = np.eye(n)
    Q = np.eye(m*n)
    R = np.eye(n)

    targets = []
    for goal in imon.prob.goals:
        target_polytopeV = goal.getVrep()
        targets.append( np.mean(target_polytopeV, axis=0) );

    current = 0
    while not rospy.is_shutdown():
        lqrc = LQRController("input", "state", A, B, targets[current], Q, R)
        while (not rospy.is_shutdown()
               and not lqrc.trial_ended
               and ((lqrc.error is None) or (la.norm(lqrc.error) > 0.01))):
            time.sleep(0.5)
        lqrc.unregister()
        if lqrc.trial_ended:
            return
        print('Reached '+imon.prob.goals[current].label)
        current += 1
        if current >= len(imon.prob.goals):
            current = 0

if __name__ == "__main__":
    rospy.init_node("lqr", anonymous=True)
    imon = InstanceMonitor()
    number_trials = rospy.get_param('/number_trials', None)
    trial_counter = 0
    while (number_trials is None) or (trial_counter < number_trials):
        main(imon)
        if number_trials is not None:
            trial_counter += 1
