"""
Illustration of the scaling double-integrators domain.

SCL; 23 Sep 2014
"""

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# The system is composed of double integrators block-wise.
#
# Even indices are (abstract) position; odd indices are velocity for
# the position corresponding to the immediately preceding index.
def dinteg_ode(x, t):
    return (x[1], -0.5*x[0], x[3], -0.5*x[2], x[5], -x[4])


def gen_obs(offset=None, side=1):
    if offset is None:
        offset = np.zeros(3)
    return (np.array([[offset[0], offset[0], offset[0]+side, offset[0]+side, offset[0]],
                      [offset[0], offset[0], offset[0]+side, offset[0]+side, offset[0]]]),
            np.array([[offset[1]+side, offset[1]+side, offset[1]+side, offset[1]+side, offset[1]+side],
                      [offset[1], offset[1], offset[1], offset[1], offset[1]]]),
            np.array([[offset[2], offset[2]+side, offset[2]+side, offset[2], offset[2]],
                      [offset[2], offset[2]+side, offset[2]+side, offset[2], offset[2]]]))


t = np.linspace(0, 10, 100)
x = odeint(dinteg_ode, [1,1, 2,0, 3,0], t)

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.plot(x.T[0], x.T[2], x.T[4])


obstacle1 = gen_obs((-2, -1, 0))
obstacle2 = gen_obs((-1.5, -1.5, -1))
ax.plot_surface(*obstacle1, rstride=1, cstride=1, color="gray")
ax.plot_surface(*obstacle2, rstride=1, cstride=1, color="gray")

goal = gen_obs((1.5, 1.5, -3), side=0.4)
ax.plot_surface(*goal, rstride=1, cstride=1, color="green")

plt.axis("equal")
-168, 8
plt.savefig("dinteg_illustration.svg")
