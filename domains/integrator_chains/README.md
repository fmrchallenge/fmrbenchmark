Problem domain: Scaling chains of integrators
=============================================

Summary
-------

This domain involves controlling chains of integrators so that all trajectories
repeatedly reach several regions while avoiding others.


Organization of sources
-----------------------

* dynamaestro/

* analysis/ tools for evaluating and reviewing results.

* $FMRBENCHMARK/examples/integrator_chains_examples/ examples of partial or
  complete solutions.


Components of the benchmark
---------------------------




Dependencies of the benchmark
-----------------------------

* [Eigen](http://eigen.tuxfamily.org)
* [Boost](http://www.boost.org), specifically [Boost.Thread](http://www.boost.org/libs/thread/)

Note that dependencies of examples and analysis tools are *not* listed here
because they are not necessary to use the benchmark per se.


Analysis tools
--------------

There are a variety of manners of studying the results. Several tools are
included to aid in this process. These may have significant overlap with respect
to motivating use-cases, but the redundancy is justified by differing
dependencies and, accordingly, better support for a variety of platforms.

### Common Python dependencies

* numpy, which is part of the [standard scientific Python stack](http://www.scipy.org/stackspec.html)
* Matplotlib, also part of the standard stack

### analysis/plotp.py

A Python script for plotting a problem instance.

* [pycddlib](https://pypi.python.org/pypi/pycddlib), a Python wrapper for
  Komei Fukuda's [cddlib](http://www.inf.ethz.ch/personal/fukudak/cdd_home/index.html).


Examples
--------

Various examples are included under the directory fmrb_sci_examples/ to
demonstrate basic usage as well as to provide partial or complete solutions.

Along with the common Python dependencies that are listed above, the examples
require the following:

* [Python Control Systems Library](https://github.com/python-control/python-control)
