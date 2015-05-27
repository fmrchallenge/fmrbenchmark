#!/usr/bin/env python
"""Plot trajectories from, compute statistics of, etc. trial data files.

If no arguments are given, then print a brief summary.
"""
from __future__ import print_function
import sys
import argparse
import json
import numpy as np
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
    parser.add_argument('--state', type=str, metavar='I',
                        dest='state_indices', default='',
                        help=('comma-separated list of indices of state, '
                              'or "*" to indicate entire state vector.'))
    parser.add_argument('--input', type=str, metavar='I',
                        dest='input_indices', default='',
                        help=('comma-separated list of indices of input, '
                              'or "*" to indicate entire input vector.'))
    parser.add_argument('--time', action='store_true',
                        dest='plot_timeseries', default=False,
                        help=('create timeseries plot. Parts of the state '
                              'and input to include can be selected using '
                              'the `--state` and `--input` switches, '
                              'respectively.'))
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
        sys.exit(0)

    prob = integrator_chains.Problem.loadJSONdict(td['trials'][args.T]['problem_instance'])
    state_dim = prob.output_dim*prob.number_integrators

    if args.plot_timeseries:
        def get_index_list(indices_str, bounds):
            if indices_str == '*':
                indices = range(bounds[0], bounds[1]+1)
            else:
                indices = [bounds[0]+int(k) for k in indices_str.split(',')]
                if (min(indices) < bounds[0]
                    or max(indices) > bounds[1]):
                    return None
            return indices

        try:
            traj = np.array(td['trials'][args.T]['trajectory'])
        except KeyError:
            print('Trial '+str(args.T)+' does not have an associated trajectory.')
            sys.exit(-1)

        if args.state_indices.strip() == '':
            state_indices = []
        else:
            state_index_bounds = (2+prob.output_dim, 2+prob.output_dim+state_dim-1)
            state_indices = get_index_list(args.state_indices, state_index_bounds)
            if state_indices is None:
                print('One of the requested state indices is out of bounds, '
                      '[0, '+str(state_index_bounds[1]-state_index_bounds[0])+']')
                sys.exit(-1)

        if args.input_indices.strip() == '':
            input_indices = []
        else:
            input_index_bounds = (2, 2+prob.output_dim-1)
            input_indices = get_index_list(args.input_indices, input_index_bounds)
            if input_indices is None:
                print('One of the requested input indices is out of bounds, '
                      '[0, '+str(input_index_bounds[1]-input_index_bounds[0])+']')
                sys.exit(-1)

        if len(state_indices) == 0 and len(input_indices) == 0:
            print('Try `--state` and `--input` options to select indices for plotting.')
            sys.exit(0)

        t = traj[:,0]+traj[:,1]*1e-9
        t -= t[0]
        if len(state_indices) > 0:
            fig = plt.figure()
            ax = fig.add_subplot(111)
            ax.hold(True)
            for state_ind in state_indices:
                ax.plot(t, traj.T[state_ind])
            ax.set_title('state')
            ax.set_xlabel('Time (s)')
            ax.legend([str(k-state_index_bounds[0]) for k in state_indices])
        if len(input_indices) > 0:
            fig = plt.figure()
            ax = fig.add_subplot(111)
            ax.hold(True)
            for state_ind in input_indices:
                ax.plot(t, traj.T[state_ind])
            ax.set_title('input')
            ax.set_xlabel('Time (s)')
            ax.legend([str(k-input_index_bounds[0]) for k in input_indices])
        plt.show()

    else:
        print(json.dumps(td['trials'][args.T]))
