#!/usr/bin/env python
"""Run trials; collect results.
"""
from __future__ import print_function
import argparse
import random
import json
import tempfile
import subprocess
import os

from fmrb import dubins_traffic


def gen_roslaunch(worldsdf_filename, rnd_path):
    nl = '\n'
    idt = ' '*2
    output = '<launch>'+nl
    output += '<include file="$(find gazebo_ros)/launch/empty_world.launch">'
    output += '<arg name="world_name" value="'+worldsdf_filename+'" />'
    output += """
    <arg name="headless" value="false" />
    <arg name="gui" value="true" />
  </include>

  <param name="robot_description" command="$(find xacro)/xacro.py '$(find dub_sim)/urdf/lasermounted.urdf.xacro'" />
"""
    for eagent_index in range(2):
        output += """
  <include file="$(find dub_sim)/launch/includes/scopedbase.launch.xml">
    <arg name="namespace" value="vehicle{EAGENT_INDEX}" />
    <arg name="init_pose" value="-x {X} -y {Y} -z 0 -Y 0" />
  </include>
  <node pkg="wander" type="wander"
   name="$(anon vehicle{EAGENT_INDEX})" ns="vehicle{EAGENT_INDEX}"
   args="{RNDPATH}">
    <remap from="cmd_vel" to="mobile_base/commands/velocity" />
  </node>
""".format(EAGENT_INDEX=eagent_index,
           X=int(random.random()*10), Y=int(random.random()*10),
           RNDPATH=rnd_path)

    return output+nl+'</launch>'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('FILE', type=str, help='road network description file')
    args = parser.parse_args()

    with open(args.FILE, 'rt') as f:
        roads = dubins_traffic.RoadNetwork(f)

    tempfd_sdf, tempfname_sdf = tempfile.mkstemp()
    worldsdffile = os.fdopen(tempfd_sdf, 'w+')
    worldsdffile.write(dubins_traffic.gen_worldsdf(roads))
    worldsdffile.close()

    tempfd, tempfname = tempfile.mkstemp()
    launchfile = os.fdopen(tempfd, 'w+')
    launchfile.write(gen_roslaunch(tempfname_sdf, rnd_path=args.FILE))
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
