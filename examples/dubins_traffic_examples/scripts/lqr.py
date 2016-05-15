#!/usr/bin/env python
"""

Some parts are copied from or based on code by SCL originally
developed for the course ME132a at Caltech, which is available in the
public domain, or equivalent in jurisdictions that do not allow it.
"""
from __future__ import print_function
import time
import numpy as np
import numpy.linalg as la

import roslib; roslib.load_manifest('dubins_traffic_examples')
import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from integrator_chains_msgs.msg import ProblemInstanceJSON
from dubins_traffic_msgs.srv import MMode, MModeRequest

from fmrb import dubins_traffic


class OdomWatcher:
    def __init__(self, topic_name='odom'):
        self.odomsub = rospy.Subscriber(topic_name, Odometry, self.odom_cb,
                                        queue_size=1)
        self.last_opose = None

    def odom_cb(self, od):
        q = [od.pose.pose.orientation.x,
             od.pose.pose.orientation.y,
             od.pose.pose.orientation.z,
             od.pose.pose.orientation.w]
        self.last_opose = (od.pose.pose.position.x, od.pose.pose.position.y,
                           euler_from_quaternion(q)[2])

    def get_pose_estimate(self):
        """Get current pose estimate

        We assume a point in SE(3) given in the local coordinates as a
        point in R^2 \times [-pi, pi]. The return type is tuple if an
        estimate is available. Otherwise, return None.
        """
        return self.last_opose


class InstanceMonitor:
    def __init__(self, problemJSON_topic='/dubins_traffic_maestro/probleminstance_JSON'):
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

    pose_watcher = OdomWatcher('odom')
    action = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)

    # Request for trial to start, and wait for maestro to declare it.
    mmode = rospy.ServiceProxy("/dubins_traffic_maestro/mode", MMode)
    while not mmode(MModeRequest.READY):
        time.sleep(0.5)
    assert mmode(MModeRequest.START)

    # Wait for problem instance description.
    while imon.prob is None:
        time.sleep(0.01)

    waypoints = []
    current_index = 0
    for goal in imon.prob.goals:
        waypoints.append(imon.prob.rnd.map_point(goal[0], goal[1]))
    waypoints = np.array(waypoints)

    forward_speed = 0.5
    turning_rate = 0.9
    min_angle_err = 0.1

    rate = rospy.Rate(30)
    while not rospy.is_shutdown() and not imon.trial_ended:
        current_pose = pose_watcher.get_pose_estimate()
        mot = Twist()
        if current_pose is not None:
            angle_diff = np.arctan2(waypoints[current_index][1]-current_pose[1],
                                    waypoints[current_index][0]-current_pose[0]) - current_pose[2]
            if la.norm(waypoints[current_index]-current_pose[:2]) < imon.prob.intersection_radius/2.0:
                print("Reached waypoint "+str(current_index))
                current_index += 1
                if current_index >= waypoints.shape[0]:
                    current_index = 0
            elif np.abs(angle_diff) > min_angle_err:
                if angle_diff > 0:
                    mot.angular.z = turning_rate
                else:
                    mot.angular.z = -turning_rate
            else:
                mot.linear.x = forward_speed
        action.publish(mot)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("lqr", anonymous=True)
    imon = InstanceMonitor()
    number_trials = rospy.get_param('/number_trials', None)
    trial_counter = 0
    while (number_trials is None) or (trial_counter < number_trials):
        main(imon)
        if number_trials is not None:
            trial_counter += 1
