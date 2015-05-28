#!/usr/bin/env python
"""Depict problem instance that has 2-dimensional output space.

If given `-`, then read from stdin. If no filename is given at
command-line, then try to get description from ROS Parameter Server.
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
    if prob.output_dim != 2:
        print('This script can only plot for output space dimension of 2, '
              'but that of given problem has dimension '
              +str(prob.output_dim)+'.')
        sys.exit(0)

    ax = plt.axes()
    ax.axis(prob.Y.get_bbox()[:4])
    prob.plot(ax)
    plt.show()
