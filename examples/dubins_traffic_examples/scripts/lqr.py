#!/usr/bin/env python
from __future__ import print_function
import time

import roslib; roslib.load_manifest('dubins_traffic_examples')
import rospy
from integrator_chains_msgs.msg import ProblemInstanceJSON
from dubins_traffic_msgs.srv import MMode, MModeRequest

from fmrb import dubins_traffic


class InstanceMonitor:
    def __init__(self, problemJSON_topic='dubins_traffic_maestro/probleminstance_JSON'):
        self.instanceJSONsub = rospy.Subscriber(problemJSON_topic,
                                                ProblemInstanceJSON,
                                                self.get_newinstance, queue_size=1)
        self.last_updated = None
        self.prob = None
        self.busy = False
        self.trial_ended = False

    def get_newinstance(self, pinst):
        self.busy = True
        self.last_updated = pinst.stamp
        if len(pinst.problemjson) == 0:
            self.trial_ended = True
            self.prob = None
        else:
            self.trial_ended = False
            self.prob = dubins_traffic.Problem.loadJSON(pinst.problemjson)
        self.busy = False


def main(imon):
    imon.trial_ended = False
    start_time = rospy.Time.now()
    mmode = rospy.ServiceProxy("dubins_traffic_maestro/mode", MMode)
    while not mmode(MModeRequest.READY):
        time.sleep(0.5)
    assert mmode(MModeRequest.START)

    while not rospy.is_shutdown() and not imon.trial_ended:
        time.sleep(0.5)


if __name__ == "__main__":
    rospy.init_node("lqr", anonymous=True)
    imon = InstanceMonitor()
    number_trials = rospy.get_param('/number_trials', None)
    trial_counter = 0
    while (number_trials is None) or (trial_counter < number_trials):
        main(imon)
        if number_trials is not None:
            trial_counter += 1
