#!/usr/bin/env python
"""

If given `-`, then read from stdin.
"""
from __future__ import print_function
import sys
import matplotlib.pyplot as plt

from fmrb import integrator_chains


if __name__ == '__main__':
    if '-h' in sys.argv or len(sys.argv) > 2:
        print('Usage: plotp.py [FILE]')
        sys.exit(0)
    elif len(sys.argv) == 1:
        param_name = '/dynamaestro/probleminstance'
        try:
            import rospy
            probjs = rospy.get_param(param_name)
        except:
            print('Failed to get "'+param_name+'" from ROS Parameter Server')
            sys.exit(-1)
    else:
        if sys.argv[1] == '-':
            f = sys.stdin
        else:
            f = open(sys.argv[1], 'r')
        probjs = f.read()
        if f is not sys.stdin:
            f.close()

    prob = integrator_chains.Problem.loadJSON(probjs)
    print('Discretization period is '+str(prob.period))

    ax = plt.axes()
    ax.axis(prob.Y.get_bbox()[:4])
    prob.plot(ax)
    plt.show()
