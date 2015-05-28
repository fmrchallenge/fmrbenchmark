Problem domain: Traffic network of Dubins cars
==============================================

Summary
-------

This domain involves navigation in a small network of two-lane roads with
vehicles that have Dubins car dynamics.


Installation
------------

A good start is the "desktop-full" configuration of ROS Indigo.  Additional
dependencies:
* [kobuki_node](http://wiki.ros.org/kobuki_node) and dependencies.
* [kobuki_description](http://wiki.ros.org/kobuki_description) and dependencies.

If you are on Ubuntu 14.04, try

    apt-get install ros-indigo-kobuki-node ros-indigo-kobuki-description


### Simulation variant

The installation process assumes a workspace named `dubplay`.  These
instructions can be easily modified for your particular setting.  Begin by
creating the workspace directory, and then link to your copy of the ROS package
`dub_sim`, which is available in the fmrbenchmark repository at the same level
as this README file.

    mkdir -p dubplay/src
    cd dubplay/src
    ln -s /you/path/to/dub_sim

At this step you should be in the root of your workspace, i.e., the directory
named `dubplay`.  Finally build everything, and launch the straightroad example.

    catkin_make
    source devel/setup.zsh
    roslaunch dub_sim straightroad.launch

where the `source` command assumes that you are using the Z shell; try
`setup.bash` if you use Bash.


References
----------

* [upstream Kobuki Gazebo plugin](http://wiki.ros.org/kobuki_gazebo_plugins)
