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


def get_summary(td, include_trials=False):
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
    if include_trials:
        for k in range(len(td['trials'])):
            summary += 'trial '+str(k)+':'+nl
            prob = integrator_chains.Problem.loadJSONdict(td['trials'][k]['problem_instance'])

            summary += idt+'output dimensions: '+str(prob.output_dim)+nl
            start_time = td['trials'][k]['start_time'][0]+td['trials'][k]['start_time'][1]*1e-9

            summary += idt+'nominal duration (s): '+str(td['trials'][k]['duration'])+nl

            summary += idt
            try:
                decision_time = td['trials'][k]['decision_time'][0]+td['trials'][k]['decision_time'][1]*1e-9
                decision_diff = decision_time - start_time
                summary += 'decision duration (s): {:.4}'.format(decision_diff)
            except KeyError:
                summary += 'no decision time included.'
            summary += nl

            if td['trials'][k].has_key('trajectory'):
                second_state_time = td['trials'][k]['trajectory'][0][0]+td['trials'][k]['trajectory'][0][1]*1e-9
                end_state_time = td['trials'][k]['trajectory'][-1][0]+td['trials'][k]['trajectory'][-1][1]*1e-9
                trajectory_duration = end_state_time - second_state_time
                summary += idt+'trajectory duration (s): {:.4}'.format(trajectory_duration)+nl

            summary += idt
            try:
                summary += 'realizable: '+str(td['trials'][k]['realizable'])
            except KeyError:
                summary += 'no declaration of realizability included.'
            summary += nl

            summary += idt
            try:
                summary += 'discrete length of trajectory: '+str(len(td['trials'][k]['trajectory']))
            except KeyError:
                summary += 'no trajectory included.'
            summary += 2*nl

    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('FILE', type=str)
    parser.add_argument('-s', action='store_true',
                        dest='per_trial_summary', default=False,
                        help='print summary for every trial.')
    parser.add_argument('-t', '--trial', type=int, dest='T',
                        help=('probe trial T; use with other trial-specific'
                              ' arguments or (default) get trial data in JSON.'))
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
        print(get_summary(td, args.per_trial_summary))
        sys.exit(0)

    if args.get_probinstance:
        print(json.dumps(td['trials'][args.T]['problem_instance']))
    else:
        print(json.dumps(td['trials'][args.T]))
