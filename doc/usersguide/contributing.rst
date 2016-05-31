Contributing
============

There are many ways to contribute. Major concerns to keep in mind:

* Participants should adhere to `the Debian Code of Conduct
  <https://www.debian.org/code_of_conduct>`_. (Replace references to "Debian"
  with "fmrbenchmark" and "fmrchallenge" as appropriate.)

* Our mailing list is `fmrbenchmark-users@googlegroups.com
  <https://groups.google.com/forum/#!forum/fmrbenchmark-users>`_.
  There is also a low-volume  `announcements newsletter <http://eepurl.com/bbxEcz>`_.

* You must hold the copyright or have explicit permission from the copyright
  holder for anything that you contribute. Furthermore, to be included in this
  project, your contributed works must be under the standard "BSD 3-clause
  license" or a comparable open-source license (including public domain
  dedication). You can find a copy at ``LICENSE`` in the root of the
  repository. A license is "comparable" if it is no more restrictive than the
  `Apache License, Version 2.0 <http://opensource.org/licenses/Apache-2.0>`_.

Please report potential bugs or request features using the `issue tracker
<https://github.com/fmrchallenge/fmrbenchmark/issues>`_. Guidelines for
participating in development are given in :doc:`developersguide`.


Proposing benchmarks
--------------------

Proposals about benchmark problems or supporting infrastructure are always
welcome and need not have a demonstrating implementation. Furthermore, in your
proposal you can use an implementation that is not ready for immediate inclusion
in the repository, e.g., if it is created entirely in MATLAB. Such
implementations are still useful because they provide a reference about your
original intent and can be a basis for porting, e.g., to C++ or Python. In most
cases, there are three parts involved in the inclusion of a benchmark:

1. a normative description about the problem and methods of evaluation in the
   `Challenge Document <https://fmrchallenge.org/norm>`_;
2. introductory and tutorial treatment in the `User's Guide <http://docs.fmrchallenge.org>`_,
   and relevant additions to the `API manual <http://api.fmrchallenge.org>`_;
3. details and practical considerations for using it as part of a competition.


Please report potential bugs or request features using the `issue tracker
<https://github.com/fmrchallenge/fmrbenchmark/issues>`_.


Working on physical variants of the problem domains
---------------------------------------------------

One of our ambitions is to create benchmarks that involve physical systems. In
other words, we want to create well-documented testbeds that facilitate
repeatability of published experiments involving real robot hardware and are
challenging with respect to the state of the art.

There are a lot of incidental costs and resource requirements to develop
physical benchmarks, such as raw materials, lab space, etc. Usually these are
provided by each lab group for their own internal purposes (often with little or
no public disclosure of details). However, this project is a joint effort that
is not under the purview of a single grant nor institution. Thus an important
manner of contribution is to realize physical variants of the benchmarks in your
own lab and then give feedback about missing details, subtle considerations,
etc. Any of the venues listed above (at the beginning of :doc:`contributing`)
can be used to provide comments. Also, the authors can be `emailed directly
<https://fmrchallenge.org/#contact>`_.


Providing computing resources
-----------------------------

Two important aspects of benchmarking are scale and comparability of performance
results. Several of the domains are designed to have problem instances that can
be arbitrarily large, e.g., :doc:`integrator_chains`. To support these
ambitions, we accept donations of hardware as well as of remote access to
computing resources, e.g., university-managed clusters or cloud computing
services.
