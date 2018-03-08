#!/usr/bin/env python
"""Plot trajectories from, compute statistics of, etc. trial data files.

If no arguments are given, then print a brief summary.
"""
from __future__ import print_function
import sys
import argparse
import json
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

from fmrb import integrator_chains

def get_labeling(td, trial_index, modrep=False):
    try:
        traj = np.array(td['trials'][trial_index]['trajectory'])
    except KeyError:
        print('Trial '+str(trial_index)+' does not have an associated trajectory.')
        sys.exit(-1)
    prob = integrator_chains.Problem.loadJSONdict(td['trials'][trial_index]['problem_instance'])
    word = []
    for row in traj:
        y = row[2:(2+prob.output_dim)]
        y_label = []
        for lpolytope in prob.goals+prob.obstacles:
            if lpolytope.contains(y):
                if lpolytope.label not in y_label:
                    y_label.append(lpolytope.label)
        if not modrep or (len(word) == 0 or word[-1] != y_label):
            word.append(y_label)
    return word


def plot(self, ax, outdims=None):
    if outdims is None:
        outdims = [0, 1]
    self.Y.plot(ax, color=(1,1,1))
    ax.hold(True)
    for lpoly in self.goals+self.obstacles:
        lpoly.plot(ax, alpha=0.8)
        xc = np.mean(lpoly.getVrep(), axis=0)
        ax.text(xc[0], xc[1], lpoly.label)
    ax.plot(self.Xinit[outdims[0]], self.Xinit[outdims[1]], 'o')


def plot_trajectory(td, trial_index, state_indices):
    probjs = td['trials'][trial_index]['problem_instance']
    prob = integrator_chains.Problem.loadJSONdict(probjs)
    state_dim = prob.output_dim*prob.number_integrators

    try:
        traj = np.array(td['trials'][trial_index]['trajectory'])
    except KeyError:
        print('Trial '+str(trial_index)+' does not have an associated trajectory.')
        sys.exit(-1)

    if state_indices.strip() == '':
        state_indices = []
    else:
        state_index_bounds = (2+prob.output_dim, 2+prob.output_dim+state_dim-1)
        state_indices = _get_index_list(state_indices, state_index_bounds)
        if state_indices is None:
            print('One of the requested state indices is out of bounds, '
                  '[0, '+str(state_index_bounds[1]-state_index_bounds[0])+']')
            sys.exit(-1)

    if len(state_indices) == 0:
        print('Try `--state` option to select indices for plotting.')
        sys.exit(0)

    probjsCorrected = json.dumps(probjs)
    probPlot = integrator_chains.Problem.loadJSON(probjsCorrected)

    x = traj[:,state_indices]
    # 1D
    if len(state_indices) == 1:
        plt.plot(x.T[0], 'r.-')
        plt.plot(x.T[0][0], 'r*')
        plt.show()

    # 2D
    elif len(state_indices) == 2:
        ax = plt.axes()
        probPlot.Y.plot(ax, color=(1,1,1))
        ax.hold(True)
        for lpoly in probPlot.goals+probPlot.obstacles:
            lpoly.plot(ax, alpha=0.6)
            xc = np.mean(lpoly.getVrep(), axis=0)
            ax.text(xc[0], xc[1], lpoly.label)
        ax.plot(probPlot.Xinit[0], probPlot.Xinit[1], 'o')
        plt.plot(x.T[0], x.T[1], 'r.-')
        plt.plot(x.T[0][0], x.T[1][0], 'r*')
        plt.show()

    # 3D
    elif len(state_indices) == 3:
        import os
        os.environ["ETS_TOOLKIT"] = "qt4"
        from mayavi import mlab
        from tvtk.tools import visual
        from tvtk.api import tvtk
        from tvtk.common import configure_input_data

        f = mlab.figure(size=(500,500))
        # Tell visual to use this as the viewer.
        visual.set_viewer(f)

        # mlab.plot3d() of Mayavi fails if there are repetitions of rows,
        # so we first manually delete any.
        y = np.zeros((x.shape[0], 3))
        y[0] = x[0]
        i = 1
        for j in range(1, x.shape[0]):
            if (x[j] != x[j-1]).any():
                y[i] = x[j]
                i += 1
        y = y[:(i+1),:]
        y = y[0:(len(y)-1)]

        mlab.plot3d(y.T[0], y.T[1], y.T[2], color=(1.0, 0.0, 0.0))
        mlab.points3d(y.T[0][0], y.T[1][0], y.T[2][0], color=(1.0, 0.0, 0.0), mode='sphere', opacity=1.0, scale_factor=0.5)

        for lpoly in probPlot.obstacles:
            xc = np.mean(lpoly.getVrep(), axis=0)
            bbox = lpoly.get_bbox()
            aObs = visual.box(x=xc[0], y=xc[1], z=xc[2], length=(bbox[1]-bbox[0]), height=(bbox[3]-bbox[2]), width=(bbox[5]-bbox[4]), color=(1.0, 0.5, 0.5))
            aObs.actor.property.opacity = 0.6

        for lpoly in probPlot.goals:
            xc = np.mean(lpoly.getVrep(), axis=0)
            bbox = lpoly.get_bbox()
            aGoal = visual.box(x=xc[0], y=xc[1], z=xc[2], length=(bbox[1]-bbox[0]), height=(bbox[3]-bbox[2]), width=(bbox[5]-bbox[4]), color=(0.0, 1.0, 0.0))
            aGoal.actor.property.opacity = 0.2

        mlab.show()

    else:
        print('Plotting of trajectories requires selection of 2 or 3 indices.'
              '\nTry `--state` option to select indices for plotting.'
              '\nNow changed to work for 1D too!')
        sys.exit(0)


def plot_timeseries(td, trial_index, state_indices, input_indices):
    prob = integrator_chains.Problem.loadJSONdict(td['trials'][trial_index]['problem_instance'])
    state_dim = prob.output_dim*prob.number_integrators

    try:
        traj = np.array(td['trials'][trial_index]['trajectory'])
    except KeyError:
        print('Trial '+str(trial_index)+' does not have an associated trajectory.')
        sys.exit(-1)

    if state_indices.strip() == '':
        state_indices = []
    else:
        state_index_bounds = (2+prob.output_dim, 2+prob.output_dim+state_dim-1)
        state_indices = _get_index_list(state_indices, state_index_bounds)
        if state_indices is None:
            print('One of the requested state indices is out of bounds, '
                  '[0, '+str(state_index_bounds[1]-state_index_bounds[0])+']')
            sys.exit(-1)

    if input_indices.strip() == '':
        input_indices = []
    else:
        input_index_bounds = (2, 2+prob.output_dim-1)
        input_indices = _get_index_list(input_indices, input_index_bounds)
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

def _get_index_list(indices_str, bounds):
    if indices_str == '*':
        indices = range(bounds[0], bounds[1]+1)
    else:
        indices = [bounds[0]+int(k) for k in indices_str.split(',')]
        if (min(indices) < bounds[0]
            or max(indices) > bounds[1]):
            return None
    return indices


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
    parser.add_argument('--traj', action='store_true',
                        dest='plot_trajectory', default=False,
                        help=('create plot of trajectory. Parts of the '
                              'state to include can be selected using '
                              'the `--state` switch.'))
    parser.add_argument('--word', action='store_true',
                        dest='get_labeling', default=False,
                        help=('get labeling of the trajectory (a.k.a. the '
                              'corresponding "word") for the requested '
                              'trial. The labeling is computed here and '
                              'not part of the saved trial data. Output '
                              'is in JSON.'))
    parser.add_argument('--wordmodrep', action='store_true',
                        dest='get_labeling_norep', default=False,
                        help='like `--word` switch but without repetition.')
    args = parser.parse_args()

    with open(args.FILE, 'r') as f:
        td = json.load(f)

    if args.T is not None:
        if args.T < 0 or args.T >= len(td['trials']):
            print('Given trial number is out of the valid range, '
                  '[0,'+str(len(td['trials'])-1)+']')
            sys.exit(-1)
        if args.get_labeling and args.get_labeling_norep:
            print('Cannot use --word and --wordmodrep simultaneously.')
            sys.exit(-1)
    else:
        print(get_summary(td, args.per_trial_summary))
        sys.exit(0)

    if args.get_probinstance:
        print(json.dumps(td['trials'][args.T]['problem_instance']))
        sys.exit(0)

    if args.plot_timeseries:
        plot_timeseries(td, args.T, args.state_indices, args.input_indices)
        plt.show()
    elif args.plot_trajectory:
        plot_trajectory(td, args.T, args.state_indices)
    elif args.get_labeling:
        print(json.dumps(get_labeling(td, args.T)))
    elif args.get_labeling_norep:
        print(json.dumps(get_labeling(td, args.T, modrep=True)))
    else:
        print(json.dumps(td['trials'][args.T]))
