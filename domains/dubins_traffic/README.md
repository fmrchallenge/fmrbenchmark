Problem domain: Traffic network of Dubins cars
==============================================

Summary
-------

This domain involves navigation in a small network of two-lane roads with
vehicles that have Dubins car dynamics.


Organization of sources
-----------------------

* e-agents/ agents that implement adversarial (environment) strategies on other cars.

* dub_sim/ main code for the simulation variant, including Gazebo SDF models and
  roslaunch files.


Installation
------------

More details are provided in the [User's Guide](http://docs.fmrchallenge.org).

A good start is the "desktop-full" configuration of ROS Indigo.
If you are on Ubuntu 14.04, try

    apt-get install ros-indigo-kobuki-node ros-indigo-kobuki-description ros-indigo-kobuki-gazebo


References
----------
