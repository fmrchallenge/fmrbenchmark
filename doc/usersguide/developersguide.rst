Developer's Guide
=================

Consult :doc:`contributing`, and join the mailing list `fmrbenchmark-users@googlegroups.com
<https://groups.google.com/forum/#!forum/fmrbenchmark-users>`_.

Bugfixes and other corrections, implementations of new features, improvements to
documentation, etc. should be offered as `pull requests
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
everything else in the repository, the example itself must be under the `Apache
2.0 license`_.
If you are going to contribute examples having dependencies
that are not free as in freedom, please carefully document the special
requirements for running the example controller.


Style
-----

Eventually we may create official style guidelines, but for now, please skim the
source code to get an indication of the preferred style.


Checklist for making releases
-----------------------------

1. tag in repository, and sign it.
2. post fmrb Python package to PyPI.
3. post releases of documentation: User's Guide, API manual, and the Challenge Document.
4. update website.


.. _Apache 2.0 license: https://www.apache.org/licenses/LICENSE-2.0.html
