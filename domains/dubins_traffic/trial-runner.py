#!/usr/bin/env python
"""Run trials and collect results for the Problem domain: Traffic network of Dubins cars.
"""
from __future__ import print_function
import argparse
import random
import json
import tempfile
import subprocess
import os
import sys
from time import gmtime, strftime

from fmrb import dubins_traffic


def gen_roslaunch(trialconf, worldsdf_filename, rnd_path, results_filename=None):
    nl = '\n'
    idt = ' '*2
    output = '<launch>'+nl
    output += '<include file="$(find gazebo_ros)/launch/empty_world.launch">'
    output += '<arg name="world_name" value="'+worldsdf_filename+'" />'
    output += """
    <arg name="headless" value="true" />
    <arg name="gui" value="false" />
    <arg name="paused" value="true" />
    <arg name="use_sim_time" value="false" />
  </include>

  <node pkg="dubins_traffic_utils" type="monitor" name="$(anon monitor)" output="log">
    <remap from="$(anon monitor)/loutput" to="loutput" />
  </node>

  <param name="robot_description" command="$(find xacro)/xacro.py '$(find dub_sim)/urdf/lasermounted.urdf.xacro'" />

  <param name="dubins_traffic/rnd" textfile="{RND_PATH}" />

  <include file="$(find dub_sim)/launch/includes/scopedbase.launch.xml">
    <arg name="namespace" value="ego" />
    <arg name="init_pose" value="-x 0 -y 0 -z 0 -Y 0" />
  </include>
""".format(RND_PATH=rnd_path)

    try:
        output += idt+'<param name="number_trials" value="'+str(trialconf['number_trials'])+'" />'
    except KeyError:
        pass

    output += idt+'<node pkg="dubins_traffic_utils" type="maestro" name="dubins_traffic_maestro" output="screen">'
    for key in ['duration_bounds', 'number_goals_bounds']:
        try:
            output += 2*idt+'<param name="'+key+'" value="'+' '.join([str(x) for x in trialconf[key]])+'" />'+nl
        except KeyError:
            pass

    output += idt+'</node>'

    if results_filename is not None:
        results_path = os.path.abspath(results_filename)
        output += '  <param name="dubins_traffic/results_file" value="'+results_path+'" />\n'

    if 'e-agents' in trialconf:
        eagent_names = []
        for eagent in trialconf['e-agents']:
            eagent_names.append(eagent['name'])
            output += """
  <include file="$(find dub_sim)/launch/includes/scopedbase.launch.xml">
    <arg name="namespace" value="{EAGENT_NAME}" />
    <arg name="init_pose" value="-x {X} -y {Y} -z 0 -Y 0" />
  </include>
  <node pkg="{EAGENT_PKG}" type="{EAGENT_TYPE}"
   name="$(anon {EAGENT_NAME})" ns="{EAGENT_NAME}"
   args="{RNDPATH}" output="log">
    <remap from="cmd_vel" to="mobile_base/commands/velocity" />
  </node>
""".format(EAGENT_NAME=eagent['name'],
           EAGENT_PKG=eagent['type'].split('/')[0],
           EAGENT_TYPE='/'.join(eagent['type'].split('/')[1:]),
           X=int(random.random()*10), Y=int(random.random()*10),
           RNDPATH=rnd_path)
        output += '<param name="dubins_traffic/e_agents" value="{0}" />\n'.format(','.join(eagent_names))

    else:
        # Empty implies no e-agents.
        output += '<param name="dubins_traffic/e_agents" value="" />\n'

    return output+nl+'</launch>'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('FILE', type=str, help='trials configuration file')
    parser.add_argument('-f', type=str,
                        dest='DATAFILE', default=None,
                        help=('name of file in which to save trial data. '
                              'If not given, then data will not be saved. '
                              'If the file exists, quit without modifying it.'))
    parser.add_argument('-F', type=str, metavar='DATAFILE',
                        dest='DATAFILE_fuerte', default=None,
                        help=('like `-f` switch but overwrite the file '
                              'if it already exists.'))
    parser.add_argument('--rnd', type=str, metavar='FILE', dest='rnd_path',
                        help=('road network description file. The path given '
                              'here will override the road network '
                              'description or path in the configuration file '
                              '(if any).'))
    args = parser.parse_args()

    if (args.DATAFILE is not None) and os.path.exists(args.DATAFILE):
        print('Requested file "'+args.DATAFILE+'" already exists.')
        print('(Use `-F` if you want to overwrite it.)')
        sys.exit(-1)
    if args.DATAFILE_fuerte is not None:
        args.DATAFILE = args.DATAFILE_fuerte

    with open(args.FILE, 'r') as f:
        trialconf = json.load(f)

    assert trialconf['version'] == 0, 'Unrecognized version of the dubins_traffic trials configuration format: '+str(trialconf['version'])
    assert trialconf['problem_domain'] == 'dubins_traffic', 'This trial-runner is for the dubins_traffic problem domain, but the given configuration file is for: '+str(trialconf['problem_domain'])

    if args.DATAFILE is not None:
        with open(args.DATAFILE, 'w') as f:
            f.write('{"version": 0,\n')
            f.write('"date": "'+strftime('%Y-%m-%d %H:%M:%S', gmtime())+'",\n')
            f.write('"trialconf": ')
            json.dump(trialconf, f)
            f.write(',\n')


    if args.rnd_path is not None:
        rnd_path = args.rnd_path
    elif 'rnd' not in trialconf:
        print('ERROR: Road network description not provided in trials configuration file nor at command-line.')
        sys.exit(-1)
    else:
        rnd_path = trialconf['rnd']

    if isinstance(rnd_path, dict):
        roads = dubins_traffic.RoadNetwork(rnd_path)
    else:
        os.path.abspath(rnd_path)
        with open(rnd_path, 'rt') as fp:
            roads = dubins_traffic.RoadNetwork(fp)

    tempfd_rnd, tempfname_rnd = tempfile.mkstemp()
    tmprndfile = os.fdopen(tempfd_rnd, 'w+')
    if isinstance(rnd_path, dict):
        json.dump(rnd_path, tmprndfile)
    else:
        with open(rnd_path, 'rt') as fp:
            tmprndfile.write(fp.read())
    tmprndfile.close()

    tempfd_sdf, tempfname_sdf = tempfile.mkstemp()
    worldsdffile = os.fdopen(tempfd_sdf, 'w+')
    worldsdffile.write(dubins_traffic.gen_worldsdf(roads))
    worldsdffile.close()

    tempfd, tempfname = tempfile.mkstemp()
    launchfile = os.fdopen(tempfd, 'w+')
    launchfile.write(gen_roslaunch(trialconf,
                                   tempfname_sdf, rnd_path=tempfname_rnd,
                                   results_filename=args.DATAFILE))
    launchfile.seek(0)
    try:
        launchp = subprocess.Popen(['roslaunch', '-'], stdin=launchfile)
        launchp.wait()
    except KeyboardInterrupt:
        launchp.terminate()
        launchp.wait()
    launchfile.close()
    os.unlink(tempfname)
    os.unlink(tempfname_sdf)

    if args.DATAFILE is not None:
        with open(args.DATAFILE, 'a') as f:
            f.write('}')
