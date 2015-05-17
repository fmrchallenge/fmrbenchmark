#!/usr/bin/env python
"""Plot dynamaestro/VectorStamped data from a ROS bagfile.
"""

from __future__ import print_function
import argparse

import rosbag
import numpy as np


if __name__ == '__main__':
    default_topicname = '/output'
    default_indices = '0,1'

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('FILE', type=str)
    parser.add_argument('-i, --indices', metavar='I', dest='indices',
                        type=str, default=default_indices,
                        help='comma-separated list of indices to plot; default is '+default_indices)
    parser.add_argument('-t, --topic', metavar='TOPIC', dest='topicname',
                        type=str, default=default_topicname,
                        help='name of topic from which to read VectorStamped messages; default is "'+default_topicname+'"')
    args = parser.parse_args()

    indices = [int(i) for i in args.indices.split(',')]
    assert len(indices) == 2 or len(indices) == 3, 'At most 3 indices can be selected.'

    bag = rosbag.Bag(args.FILE)
    msg_list = [m for m in bag if m[0] == args.topicname]
    x = np.array([m[1].v.point for m in msg_list])
    print('Trajectory array has shape '+str(x.shape))

    if len(indices) == 2:
        import matplotlib.pyplot as plt
        plt.plot(x.T[indices[0]], x.T[indices[1]], 'r.-')
        plt.plot(x.T[indices[0]][0], x.T[indices[1]][0], 'r*')
        plt.show()

    else:
        from mayavi import mlab

        # mlab.plot3d() of Mayavi fails if there are repetitions of rows,
        # so we first manually delete any.
        y = np.zeros((x.shape[0], 3))
        y[0] = x[0,indices]
        i = 1
        for j in range(1, x.shape[0]):
            if (x[j,indices] != x[j-1,indices]).any():
                y[i] = x[j,indices]
                i += 1
        y = y[:(i+1),:]

        mlab.plot3d(y.T[0], y.T[1], y.T[2],
                    color=(1.0, 0.0, 0.0))
        mlab.points3d(y.T[0][0], y.T[1][0], y.T[2][0],
                      color=(1.0, 0.0, 0.0), mode='sphere', opacity=0.5, scale_factor=0.2)
        mlab.show()
