#!/usr/bin/env python
"""Depict problem instance of Traffic network of Dubins cars
"""
from __future__ import print_function
import argparse
import sys
import matplotlib.pyplot as plt

from fmrb import dubins_traffic


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('FILE', type=str,
                        help=('Name of file containing instance JSON; '
                              'if `-`, then read from stdin.'))
    args = parser.parse_args()

    if args.FILE == '-':
        fp = sys.stdin
    else:
        fp = open(args.FILE, 'rt')
    probinstance = dubins_traffic.Problem.loadJSON(fp.read())
    if fp is not sys.stdin:
        fp.close()

    print('intersection radius: ', probinstance.intersection_radius)
    print('goals: ', probinstance.goals)

    ax = plt.axes()
    probinstance.rnd.plot(ax)
    plt.axis('equal')
    plt.show()
