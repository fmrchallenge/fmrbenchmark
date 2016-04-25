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
                        help='road network description file')
    args = parser.parse_args()

    rnd = dubins_traffic.RoadNetwork(args.FILE)

    ax = plt.axes()
    rnd.plot(ax)
    plt.axis('equal')
    plt.show()
