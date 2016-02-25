#!/usr/bin/env python
"""Script for issuing velocity commands for a differential drive robot to visit a series of waypoints.

SCL; 6 Mar 2015
"""
from __future__ import print_function
import rospy
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy as np
import numpy.linalg as la


min_dist = 0.5
safe_to_move = True


def laser_cb(scan):
    global safe_to_move
    for r in scan.ranges:
        if r < scan.range_min or r > scan.range_max:
            continue  # Discard invalid values
        if r < min_dist:
            safe_to_move = False
            return
    safe_to_move = True

class OdomWatcher:
    def __init__(self, topic_name='odom'):
        self.odomsub = rospy.Subscriber(topic_name, Odometry, self.odom_cb,
                                        queue_size=1)
        self.last_opose = None

    def odom_cb(self, od):
        q = [od.pose.pose.orientation.x, od.pose.pose.orientation.y, od.pose.pose.orientation.z, od.pose.pose.orientation.w]
        self.last_opose = (od.pose.pose.position.x, od.pose.pose.position.y,
                           euler_from_quaternion(q)[2])

    def get_pose_estimate(self):
        """Get current pose estimate

        We assume a point in SE(3) given in the local coordinates as a
        point in R^2 \times [-pi, pi]. The return type is NumPy
        ndarray if an estimate is available. Otherwise, return None.
        """
        if self.last_opose is not None:
            return np.array(self.last_opose)
        else:
            return None


if __name__ == "__main__":
    ############################################################
    ## List your waypoints here
    waypoints = np.array([[2, 2],
                          [4, 1],
                          [2, 3]])

    ## Configure turning rate and other motion parameters here
    turning_rate = 0.2  # rad/s
    forward_speed = 0.5  # m/s

    ## Thresholds for switching among modes of motion
    min_angle_err = 0.1  # Minimum relative angle to drive forward toward waypoint
    min_reach_err = 0.3  # Distance from waypoint before it is declared as "reached"

    ############################################################

    rospy.init_node('Oscar', anonymous=True)

    action = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    lmssub = rospy.Subscriber('scan', LaserScan, laser_cb, queue_size=10)
    pose_watcher = OdomWatcher()

    current_index = 0  # Visit waypoints in order

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        current_pose = pose_watcher.get_pose_estimate()

        mot = Twist()

        if safe_to_move and (current_pose is not None):
            angle_diff = np.arctan2(waypoints[current_index][1]-current_pose[1],
                                    waypoints[current_index][0]-current_pose[0]) - current_pose[2]

            if la.norm(waypoints[current_index]-current_pose[:2]) < min_reach_err:
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
