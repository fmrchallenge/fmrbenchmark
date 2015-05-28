Problem domain: Scaling chains of integrators
=============================================

Often referred to as "the first domain," the basic problem is to find a
controller for a given chain of integrators system so that all trajectories
repeatedly reach several regions while avoiding others.


Preparations
------------

While below we include pointers to the main websites for dependencies, many are
available via packages for your OS and may already be installed, especially if
you have ROS on Ubuntu 14.04. Supported platforms are described in the :doc:`intro`.

Dependencies
~~~~~~~~~~~~

* `Eigen <http://eigen.tuxfamily.org>`_
* `Boost <http://www.boost.org>`_, specifically `Boost.Thread <http://www.boost.org/libs/thread/>`_
  and `bind <http://www.boost.org/doc/libs/1_57_0/libs/bind/bind.html>`_.

Other
~~~~~

While not necessary to use the benchmark per se, ``plotp.py`` and ``tdstat.py``
provide a means to examine problem instances and results of trials, as
demonstrated in the tutorial below. Together with the ``fmrb`` Python package,
which is under ``tools/fmrb-pkg/`` in the repository, the following additional
dependencies are present:

* NumPy, which is part of the `standard scientific Python stack <http://www.scipy.org/stackspec.html>`_
* Matplotlib, also part of the standard stack
* `pycddlib <https://pypi.python.org/pypi/pycddlib>`_, a Python wrapper for
  Komei Fukuda's `cddlib <http://www.inf.ethz.ch/personal/fukudak/cdd_home/index.html>`_
* `Python Control Systems Library <https://github.com/python-control/python-control>`_

Once these are met, install ``fmrb`` from your copy of the repository, e.g., ::

  cd tools/fmrb-pkg
  pip install -e .

or `get it from PyPI <https://pypi.python.org/pypi/fmrb>`_, ::

  pip install fmrb


Tutorial
--------

In the below code, ``$FMRBENCHMARK`` is the absolute path to a copy of the
fmrbenchmark repository on your machine.

Demonstrations of components
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To build the "standalone" (i.e., independent of ROS) examples demonstrating
various parts of this benchmark, go to the ``dynamaestro`` directory
(``$FMRBENCHMARK/domains/integrator_chains/dynamaestro``) and then follow the
usual `CMake <http://www.cmake.org>`_ build instructions. On Unix without an
IDE, usually these are

.. code-block:: none

  mkdir build
  cd build
  cmake ..
  make

One of the resulting programs is ``genproblem``, the source of which is
``$FMRBENCHMARK/domains/integrator_chains/dynamaestro/examples/standalone/genproblem.cpp``.
The output is a problem instance in JSON. To visualize it, try

.. code-block:: none

  dynamaestro/build/genproblem | analysis/plotp.py -

from the directory ``$FMRBENCHMARK/domains/integrator_chains/``.

Controller examples
~~~~~~~~~~~~~~~~~~~

Note that the ``lqr.py`` controller requires the Python Control System Library
(``control``) and a standard scientific Python stack including NumPy.

Create a catkin workspace.

.. code-block:: none

  mkdir -p fmrb_demo/src
  cd fmrb_demo/src
  catkin_init_workspace


Create symbolic links to the ROS packages in the fmrbenchmark repository
required for this example.

.. code-block:: none

  ln -s $FMRBENCHMARK/domains/integrator_chains/dynamaestro
  ln -s $FMRBENCHMARK/examples/sci_concrete_examples

Build and install it within the workspace.

.. code-block:: none

  cd ..
  catkin_make install

Because the installation is local to the workspace, you must use ``source install/setup.zsh``
whenever a new shell session is created.

Finally, run the example using::

  roslaunch sci_concrete_examples lqr.launch

You can observe the sequence of states and control inputs using ``rostopic echo
state`` and ``rostopic echo input``, respectively. At each time increment, the
state labeling is published to the topic ``/dynamaestro/loutput`` as an array of
strings (labels) corresponding to the polytopes containing the output at that
time.
