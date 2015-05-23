Contributing
============

There are many ways to contribute. Major concerns to keep in mind:

* Participants should adhere to `the Debian Code of Conduct
  <https://www.debian.org/code_of_conduct>`_. (Replace references to "Debian"
  with "fmrbenchmark" and "fmrchallenge" as appropriate.)

* There is not a dedicated mailing list yet, but there is an `announcements
  newsletter <http://eepurl.com/bbxEcz>`_.

* You must hold the copyright or have explicit permission from the copyright
  holder for anything that you contribute. Furthermore, to be included in this
  project, your contributed works must be under the standard "BSD 3-clause
  license" or a comparable open-source license (including public domain
  dedication). You can find a copy at ``LICENSE`` in the root of the repository.

Please report potential bugs or request features using the `issue tracker
<https://github.com/fmrchallenge/fmrbenchmark/issues>`_.


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
   `Challenge Document <http://fmrchallenge.org/norm>`_;
2. introductory and tutorial treatment in the `User's Guide <http://docs.fmrchallenge.org>`_,
   and relevant additions to the `API manual <http://api.fmrchallenge.org>`_;
3. details and practical considerations for using it as part of a competition.


.. _contributing-dev-section:

Development
-----------

Please report potential bugs or request features using the `issue tracker
<https://github.com/fmrchallenge/fmrbenchmark/issues>`_. Bugfixes and other
corrections, implementations of new features, improvements to documentation,
etc. should be offered as `pull requests
<https://github.com/fmrchallenge/fmrbenchmark/pulls>`_. Patches can be submitted
through other media if you prefer, but please try to make it easy to use and
understand your proposed changes.

The benchmarks are primarily implemented in C++ and C. Unless there are strong
motivations to use a different programming language, we prefer these for
well-known reasons: they are fast, mature, standard, etc. Besides C and C++,
several core tools for analysis of results are in `Python
<https://www.python.org>`_ and rely on widely-used numerical and scientific
Python packages, among others. Observe that "tools for analysis" are not part of
the benchmarks per se.

Examples can be expressed in any programming language or depend on any tool,
including dependencies that have restrictive licenses. However, as with
everything else in the repository, the example itself must be under the standard
"BSD 3-clause license" or a comparable open-source license (including public
domain dedication). If you are going to contribute examples having dependencies
that are not free as in freedom, please carefully document the special
requirements for running the example controller.

In terms of planning, the project is currently sufficiently small to where it is
enough to have a combination of the issue tracker and direct communication via
private email or at meetings.

Style
~~~~~

Eventually we may create official style guidelines, but for now, please skim the
source code to get an indication of the preferred style.


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
etc. There is not a dedicated mailing list yet, so the best ways to contribute
here are the `issue tracker <https://github.com/fmrchallenge/fmrbenchmark/issues>`_,
as noted in the :ref:`contributing-dev-section` section, and via `email to the
authors <http://fmrchallenge.org/#contact>`_.

Providing computing resources
-----------------------------

Two important aspects of benchmarking are scale and comparability of performance
results. Several of the domains are designed to have problem instances that can
be arbitrarily large, e.g., :doc:`integrator_chains`. To support these
ambitions, we accept donations of hardware as well as of remote access to
computing resources, e.g., university-managed clusters or cloud computing
services.