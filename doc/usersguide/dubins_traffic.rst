Problem domain: Traffic network of Dubins cars
==============================================

Often referred to as "the second domain," the basic setting is navigation in a
small network of roads with vehicles that follow unicycle-like dynamics.
Every road network is a 4-connected grid, subject to a rigid-body transformation: as such, the segments may not be axis-aligned.


Preparations
------------

While below we include pointers to the main websites for dependencies, many are
available via packages for your OS and may already be installed, especially if
you have ROS on Ubuntu 14.04. Supported platforms are described in the :doc:`intro`.

Basic
~~~~~

There are two major variants of this benchmark: one based in simulation and
another on a physical testbed. We begin with preparations appropriate for both.

* `Eigen <http://eigen.tuxfamily.org>`_

On Ubuntu, Eigen can be obtained by installing the "libeigen3-dev" deb package
(https://packages.debian.org/jessie/libeigen3-dev).

Several ROS packages for the Kobuki by Yujin Robot are required.

* `kobuki_node <http://wiki.ros.org/kobuki_node>`_ and dependencies.
* `kobuki_description <http://wiki.ros.org/kobuki_description>`_ and dependencies.

Install ``fmrb`` from your copy of the repository, e.g., ::

  cd tools/fmrb-pkg
  pip install -e .

or `get it from PyPI <https://pypi.python.org/pypi/fmrb>`_, ::

  pip install fmrb

Dependencies of the simulation variant
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* `Gazebo <http://gazebosim.org>`_
* `kobuki_gazebo_plugins <http://wiki.ros.org/kobuki_gazebo_plugins>`_

Dependencies of the physical variant
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

(forthcoming)

Supplementary prerequisites
~~~~~~~~~~~~~~~~~~~~~~~~~~~

As for the :doc:`integrator_chains`, there is code that is relevant but not
required for this benchmark.

Teleoperation of the vehicle to be controlled can be achieved using
`kobuki_keyop <http://wiki.ros.org/kobuki_keyop>`_ ROS package. An example
demonstrating a configuration known to work in the simulation variant::

  roslaunch dubins_traffic_utils teleop.launch


Tutorials
---------

In the below code, ``$FMRBENCHMARK`` is the absolute path to a copy of the
fmrbenchmark repository on your machine.

Launching a problem instance of the simulation variant
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create a catkin workspace.

.. code-block:: none

  mkdir -p dubsim_workspace/src
  cd dubsim_workspace/src
  catkin_init_workspace

Create symbolic links to the ROS packages in the fmrbenchmark repository
required for this example.

.. code-block:: none

  ln -s $FMRBENCHMARK/domains/integrator_chains/integrator_chains_msgs
  ln -s $FMRBENCHMARK/domains/dubins_traffic/dubins_traffic_msgs
  ln -s $FMRBENCHMARK/domains/dubins_traffic/dubins_traffic_utils
  ln -s $FMRBENCHMARK/domains/dubins_traffic/dub_sim
  ln -s $FMRBENCHMARK/domains/dubins_traffic/e-agents/wander
  ln -s $FMRBENCHMARK/examples/dubins_traffic_examples


Build and install it within the catkin workspace.

.. code-block:: none

  cd ..
  catkin_make install

Because the installation is local to the catkin workspace, before beginning and
whenever a new shell session is created, you must first ::

  source install/setup.zsh

where the ``source`` command assumes that you are using the Z shell; try
``setup.bash`` if you use Bash.

Finally, launch a small 4-grid road network with two adversarially controlled
vehicles, also known as e-agents (where ``e'' abbreviates ``environment''). ::

  python $FMRBENCHMARK/domains/dubins_traffic/trial-runner.py -f mydata.json $(rospack find dubins_traffic_utils)/examples/trialsconf/mc-small-4grid-agents2.json

This will cause trial data to be saved to the file ``mydata.json`` in the local directory from where the above command is executed.

The Gazebo server is launched without a GUI frontend, which is also known as
running headless.
A local viewer can be launched using ::

  gzclient

The vehicle to be controlled has the ROS namespace ``/ego``. The e-agents have
namespaces defined in the trials configuration file. In the example
mc-small-4grid-agents2.json used in this tutorial, these are ``/agent0`` and
``/agent1``.

In a separate terminal, run your controller. For example, assuming your controller
is contained in the package ``your_controller`` with launch file ``foo.launch``,
in a separate terminal, run ::

  roslaunch your_controller foo.launch

You can run an example controller using::

  roslaunch dubins_traffic_examples simple.launch

This is a simple controller that sets the ego vehicle's forward and angular velocity based on the next goal to be visited, and cycles through goals in this manner.

Support code for working with road network descriptions is available in
``roadnet.hpp`` and ``dubins_traffic.py``.

For example, try

.. code-block:: none

  analysis/plotp.py dubins_traffic_utils/examples/data/square.json

from the directory ``$FMRBENCHMARK/domains/dubins_traffic/``.
