#!/usr/bin/env python
"""Plot trajectories from, compute statistics of, etc. trial data files.

If no arguments are given, then print a brief summary.
"""
from __future__ import print_function
import sys
import argparse
import json
import matplotlib.pyplot as plt

from fmrb import integrator_chains


def get_summary(td):
    nl = '\n'
    idt = '\t'
    assert td['version'] == 0
    summary = 'version: '+str(td['version'])+nl
    summary += 'date: '+td['date']+nl
    try:
        summary += 'extra: '+td['extra']+nl
    except KeyError:
        summary += 'extra: (none)'+nl
    summary += 'number of trials: '+str(len(td['trials']))+nl
    for k in range(len(td['trials'])):
        summary += 'trial '+str(k)+':'+nl

        summary += idt
        try:
            summary += 'realizable: '+str(td['trials'][k]['realizable'])
        except KeyError:
            summary += 'no declaration of realizability included.'
        summary += nl

        summary += idt
        try:
            summary += 'length of trajectory: '+str(len(td['trials'][k]['trajectory']))
        except KeyError:
            summary += 'no trajectory included.'
        summary += nl

    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('FILE', type=str)
    parser.add_argument('-t', '--trial', type=int, dest='T',
                        help=('probe trial T; use with other trial-specific'
                              ' arguments or (default) get details.'))
    parser.add_argument('-p', action='store_true',
                        dest='get_probinstance', default=False,
                        help='extract problem instance JSON for trial.')
    args = parser.parse_args()

    with open(args.FILE, 'r') as f:
        td = json.load(f)

    if args.T is not None:
        if args.T < 0 or args.T >= len(td['trials']):
            print('Given trial number is out of the valid range, '
                  '[0,'+str(len(td['trials'])-1)+']')
            sys.exit(-1)
    else:
        print(get_summary(td))
        sys.exit(0)

    prob = integrator_chains.Problem.loadJSONdict(td['trials'][args.T]['problem_instance'])

    if args.get_probinstance:
        print(json.dumps(td['trials'][args.T]['problem_instance']))
