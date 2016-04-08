Problem domain: Traffic network of Dubins cars
==============================================

Often referred to as "the second domain," the basic setting is navigation in a
small network of roads with vehicles that follow unicycle-like dynamics.


Preparations
------------

While below we include pointers to the main websites for dependencies, many are
available via packages for your OS and may already be installed, especially if
you have ROS on Ubuntu 14.04. Supported platforms are described in the :doc:`intro`.

Dependencies
~~~~~~~~~~~~

* `Eigen <http://eigen.tuxfamily.org>`_
* `Gazebo <http://gazebosim.org>`_

On Ubuntu, Eigen can be obtained by installing the "libeigen3-dev" deb package
(https://packages.debian.org/jessie/libeigen3-dev).

Several ROS packages for the Kobuki by Yujin Robot are required.

* `kobuki_node <http://wiki.ros.org/kobuki_node>`_ and dependencies.
* `kobuki_description <http://wiki.ros.org/kobuki_description>`_ and dependencies.
* `kobuki_gazebo_plugins <http://wiki.ros.org/kobuki_gazebo_plugins>`_

Supplementary prerequisites
~~~~~~~~~~~~~~~~~~~~~~~~~~~

As for the :doc:`integrator_chains` benchmark, there is code that is relevant
but not required for this benchmark.
