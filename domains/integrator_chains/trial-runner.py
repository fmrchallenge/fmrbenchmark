#!/usr/bin/env python
"""Run trials and collect results for the Problem domain: Scaling chains of integrators.
"""

from __future__ import print_function
import argparse
import json
import tempfile
import subprocess
import sys
import os
import os.path
from time import gmtime, strftime


def gen_roslaunch(trialconf, results_filename=None, launch_logger=False):
    nl = '\n'
    idt = ' '*2
    output = '<launch>'+nl
    try:
        output += idt+'<param name="number_trials" value="'+str(trialconf['number_trials'])+'" />'
    except KeyError:
        pass

    output += """
  <node pkg="dynamaestro" name="dynamaestro" type="dm" output="screen">
    <remap from="dynamaestro/input" to="input" />
    <remap from="dynamaestro/state" to="state" />
"""

    for key in ['output_dim_bounds', 'number_integrators_bounds',
                'number_goals_bounds', 'number_obstacles_bounds',
                'period_bounds', 'Y', 'U', 'duration_bounds']:
        try:
            output += 2*idt+'<param name="'+key+'" value="'+' '.join([str(x) for x in trialconf[key]])+'" />'+nl
        except KeyError:
            pass

    if results_filename is not None:
        results_path = os.path.abspath(results_filename)
        output += 2*idt+'<param name="results_file" value="'+results_path+'" />'+nl
    output += idt+'</node>'+nl

    if launch_logger:
        output += """
  <node pkg="dynamaestro" name="logger" type="log" output="screen">
    <remap from="dynamaestro/state" to="state" />
  </node>
"""

    return output+nl+'</launch>'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('FILE', type=str, help='trials configuration file')
    parser.add_argument('-l', action='store_true',
                        dest='launch_logger', default=False,
                        help='launch the logger of the dynamaestro package.')
    parser.add_argument('-f', type=str,
                        dest='DATAFILE', default=None,
                        help=('name of file in which to save trial data. '
                              'If not given, then data will not be saved. '
                              'If the file exists, quit without modifying it.'))
    parser.add_argument('-F', type=str, metavar='DATAFILE',
                        dest='DATAFILE_fuerte', default=None,
                        help=('like `-f` switch but overwrite the file '
                              'if it already exists.'))
    args = parser.parse_args()

    if (args.DATAFILE is not None) and os.path.exists(args.DATAFILE):
        print('Requested file "'+args.DATAFILE+'" already exists.')
        print('(Use `-F` if you want to overwrite it.)')
        sys.exit(-1)
    if args.DATAFILE_fuerte is not None:
        args.DATAFILE = args.DATAFILE_fuerte

    with open(args.FILE, 'r') as f:
        trialconf = json.load(f)

    assert trialconf['version'] == 0, 'Unrecognized version of the integrator_chains trials configuration format: '+str(trialconf['version'])
    assert trialconf['problem_domain'] == 'integrator_chains', 'This trial-runner is for the integrator_chains problem domain, but the given configuration file is for: '+str(trialconf['problem_domain'])

    if args.DATAFILE is not None:
        with open(args.DATAFILE, 'w') as f:
            f.write('{"version": 0,\n')
            f.write('"date": "'+strftime('%Y-%m-%d %H:%M:%S', gmtime())+'",\n')
            f.write('"trialconf": ')
            json.dump(trialconf, f)
            f.write(',\n')

    tempfd, tempfname = tempfile.mkstemp()
    launchfile = os.fdopen(tempfd, 'w+')
    launchfile.write(gen_roslaunch(trialconf,
                                   results_filename=args.DATAFILE,
                                   launch_logger=args.launch_logger))
    launchfile.seek(0)
    try:
        launchp = subprocess.Popen(['roslaunch', '-'], stdin=launchfile)
        launchp.wait()
    except KeyboardInterrupt:
        launchp.terminate()
        launchp.wait()
    launchfile.close()
    os.unlink(tempfname)

    if args.DATAFILE is not None:
        with open(args.DATAFILE, 'a') as f:
            f.write('}')
