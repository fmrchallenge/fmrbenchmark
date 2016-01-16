#!/usr/bin/env python
from __future__ import print_function
import math
import argparse

def road_segment(x1, x2, prefix='straightroad'):
    assert len(x1) == 2 and len(x2) == 2
    
    center = ((x1[0] + x2[0])/2.0, (x1[1] + x2[1])/2.0)
    length = math.sqrt((x1[0]-x2[0])**2 + (x1[1]-x2[1])**2)
    angle = math.atan2(x2[1]-x1[1], x2[0]-x1[0])
    nl = '\n'
    idt = ' '*2
    output = '<model name="'+prefix+'">'+nl
    output += '<static>true</static>'+nl
    output += '<link name="'+prefix+'link1">'+nl
    output += '<visual name="'+prefix+'bar1">'+nl
    output += '<pose>{X} {Y} 0 0 0 {ANGLE}</pose>'.format(X=center[0], Y=center[1], ANGLE=angle)+nl
    output += '<geometry><box><size>{LENGTH} 1.2 0.001</size></box></geometry>'.format(LENGTH=length)+nl
    output += """
          <material>
	    <ambient>0.2 0.2 0.2 1</ambient>
	    <diffuse>0.2 0.2 0.2 1</diffuse>
	    <specular>0.2 0.2 0.2 1</specular>
	    <emissive>0.2 0.2 0.2 1</emissive>
	  </material>
	</visual>
      </link>
"""
    output += '<link name="'+prefix+'link2">'+nl
    output += '<visual name="'+prefix+'bar2">'+nl
    output += '<pose>{X} {Y} 0 0 0 {ANGLE}</pose>'.format(X=center[0], Y=center[1], ANGLE=angle)+nl
    output += '<geometry><box><size>{LENGTH} 0.12 0.0015</size></box></geometry>'.format(LENGTH=length)+nl
    output += """
          <material>
	    <ambient>0.5 0.5 0 1</ambient>
	    <diffuse>0.5 0.5 0 1</diffuse>
	    <specular>0.5 0.5 0 1</specular>
	    <emissive>0.5 0.5 0 1</emissive>
	  </material>
	</visual>
      </link>
"""
    return output+nl+'</model>'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('FILE', type=str, help='road network description file')
    args = parser.parse_args()

    segments = [[-2.5, 0, 5, 0],
                [5, 0, 5, 5],
                [-2.5, 0, -2.5, -4]]

    print("""<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">
    <gui><camera name="user_camera"><pose>0 0 9.5 0 1.5 0</pose></camera></gui>

    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
""")
    for i, segment in enumerate(segments):
        print(road_segment((segment[0], segment[1]),
                           (segment[2], segment[3]), prefix='segment_'+str(i)+'_'))
    print('</world></sdf>')
