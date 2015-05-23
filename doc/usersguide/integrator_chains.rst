Problem domain: Scaling chains of integrators
=============================================

Often referred to as "the first domain," the basic problem is to find a
controller for a given chain of integrators system so that all trajectories
repeatedly reach several regions while avoiding others.


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
output`` and ``rostopic echo input``, respectively. At each time increment, the
state labeling is published to the topic ``/dynamaestro/loutput`` as an array of
strings (labels) corresponding to the polytopes containing the output at that
time.
