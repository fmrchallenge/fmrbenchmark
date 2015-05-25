#!/usr/bin/env python
"""Run trials; collect results.
"""

from __future__ import print_function
import argparse
import json


def gen_roslaunch(trialconf, launch_logger=False):
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
    parser.add_argument('FILE', type=str)
    parser.add_argument('-l', action='store_true',
                        dest='launch_logger', default=False,
                        help='launch the logger of the dynamaestro package.')
    args = parser.parse_args()

    trialconf = json.loads(open(args.FILE, 'r').read())
    print(gen_roslaunch(trialconf, launch_logger=args.launch_logger))
