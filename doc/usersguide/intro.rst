Introduction
============

This page provides orientation and an overall introduction to the repository. It
is a good place to begin before studying a particular benchmark. There are two
founding ambitions of the project: to develop benchmark problems for research in
so-called "formal methods for robotics," and to create standard interfaces,
formats, etc. for expressing problems and using tools that implement methods
described in the research literature. Our effort is analogous to that of
`SMT-LIB <http://www.smt-lib.org>`_, which is for research in satisfiability
modulo theories.

There are four major kinds of entities in the repository:

1. **benchmarks**;
2. **analysis tools** for reviewing results from using benchmarks;
3. **examples** demonstrating components of benchmarks and solution controllers;
4. **documentation**.

Spanning all four of the above kinds is the *supporting infrastructure*. This
refers to header files, message formats, etc. that may be used by more than one
benchmark and that may be of independent interest, besides benchmarking.

The repository as a whole has a single version number. Depending on the eventual
pace of growth and styles of usage, we may begin to version significant
components separately. In any case, version numbers are of the form ``M.m.u``,
and changes only to ``u`` are not expected to break any current usage.

.. WARNING:: 
   The interfaces to command-line tools, the names of important ROS topics, and
   other user-level aspects of the repository may change with little warning
   until version 0.1.0. Beginning at that time, care will be taken to ensure
   backwards-compatibility and to have more gradual deprecation.


Formulation
-----------

A normative description of benchmarks as well as a development of notation and
problem formulation is given in the `Challenge Document <http://fmrchallenge.org/norm>`_.
Below is a summary.

Benchmarks are organized into *problem domains* (sometimes also called "problem
settings"), which are defined in terms of several parameters. A *problem
variant* refines the domain by constraining possible values that may be assigned
to a parameter, e.g., deciding that time can only be a multiple of a constant
(the period). Finally, a *problem instance* is defined as a particular selection
of values consistent with a problem variant. The instance is the thing that is
actually to be solved. A special case of this taxonomy is a concrete benchmark
from industry that is to be solved as given, i.e., there is no need to provide
more details like how time progresses or what the initial state can be. In such
a case, the problem domain, variant, and instance are all the same.


Support for platforms and programming languanges
------------------------------------------------

There are no generic installation instructions. Instead, instructions and
requirements are described separately for each benchmark. Though there are
shared dependencies and some similar preparations, separately treating each
facilitates users who are only interested in some parts of the repository. E.g.,
try the :doc:`integrator_chains`.

While it may be possible to build the benchmarks and infrastructure on other
platforms, the current target is `Ubuntu <http://www.ubuntu.com>`_ 14.04
running Linux x86_64 and the following:

* `ROS Indigo <http://wiki.ros.org/indigo/Installation/Ubuntu>`_
* `Gazebo <http://gazebosim.org>`_ `as used with ROS <http://wiki.ros.org/gazebo_ros_pkgs>`_

The benchmarks are primarily implemented in C++ and C. As of version 0.0.0, most
of the examples and tools for reviewing results are in C++ and `Python
<https://www.python.org>`_.
