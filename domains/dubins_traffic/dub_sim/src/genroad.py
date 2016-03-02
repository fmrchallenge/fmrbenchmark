#!/usr/bin/env python
from __future__ import print_function
import argparse

from fmrb.dubins_traffic import gen_worldsdf


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('FILE', type=str, help='road network description file')
    args = parser.parse_args()

    with open(args.FILE, 'rt') as f:
        roads = RoadNetwork(f)

    print(gen_worldsdf(roads))
