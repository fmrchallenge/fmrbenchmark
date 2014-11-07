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

`dub_sim` requires a patched version of the Kobuki Gazebo plugin.  The package
that we desire is in a repository with other ROS packages, so it must be moved
to the `src` directory after separately getting the copy of the source.  (It
will be built when `catkin_make` is invoked later.)  E.g.,

    wget https://github.com/slivingston/kobuki_desktop/archive/indigo.tar.gz
    tar -xzf indigo.tar.gz
    mv kobuki_desktop-indigo/kobuki_gazebo_plugins /your/path/to/dubplay/src

At this step you should be in the root of your workspace, i.e., the directory
named `dubplay`.  Finally build everything, and launch the straightroad example.

    catkin_make
    roslaunch dub_sim straightroad.launch


References
----------

* the patched kobuki_gazebo_plugins ROS package is available from <https://github.com/slivingston/kobuki_desktop>
* [upstream Kobuki Gazebo plugin](http://wiki.ros.org/kobuki_gazebo_plugins)
