Problem domain: Factory cart clearing
=====================================

Summary
-------

This domain concerns dexterous manipulation tasks motivated by assembly line
activities.


Installation
------------

A good start is the "desktop-full" configuration of ROS Indigo.  Additional
dependencies:
* [baxter_description](http://wiki.ros.org/baxter_description), the repository
  for which is at <https://github.com/RethinkRobotics/baxter_common>, which must
  be obtained manually, i.e., I (Scott) am not aware of official availability as
  part of ROS Indigo.  *Note* that this dependency will be automatically met as
  part of installing the Rethink Robotics `baxter_simulator` package

* [wstool](http://wiki.ros.org/wstool); on Ubuntu, try `apt-get install python-wstool`.


### Simulation variant

Consult `pandp_sim/extern/README.md` about obtaining some of the models.

For Baxter simulation, to build it within a particular workspace named
`baxplay`, try the following commands

    mkdir -p baxplay/src
    cd baxplay/src
    catkin_init_workspace
    git clone git@github.com:RethinkRobotics/baxter_simulator.git
    cd baxter_simulator
    git checkout -b indigo-devel origin/indigo-devel
    cd ..
    wstool init .
    wstool merge baxter_simulator/baxter_simulator.rosinstall
    wstool update
    cd ..
    catkin_make
    catkin_make install

In the above, we assume that you have already configured to use ROS Indigo,
e.g., by running `source /opt/ros/indigo/setup.zsh` if you use the Z shell
(`setup.bash` if you use Bash).  The repository github.com:RethinkRobotics/baxter_simulator.git
is private and requires special permissions to access.  The last step is to copy baxter.sh to the root of the workspace,

    cp src/baxter/baxter.sh .

and then edit it by:
* adding `SHELL=/bin/bash` on or near the second line (under `#!/bin/bash`);
* changing `your_ip` near line 52 to be the IP address of the machine that you
  are using, or appropriately setting `your_hostname` instead;
* changing `ros_version` near line 56 to "indigo".

To initialize the world with a Baxter robot,

    ./baxter.sh sim
    roslaunch baxter_gazebo baxter_world.launch
