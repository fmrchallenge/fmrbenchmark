Topical README files are provided in some directories.  For example,
`domains/dubins_traffic/README.md`.

Summary of organization
-----------------------

* `doc/` : Documentation sources, including benchmark APIs, the User's Guide,
  and competition rules.

* `domains/` : Problem domains, e.g., `domains/integrator_chains/` contains code
  for the scaling chains of integrators setting.

* `tools/fmrb-pkg/` : Python package providing code for both common and
  domain-specific needs.

* `examples/` : contains example controllers and other demonstrations. There is
  a ROS meta-package for each problem domain that has name of the form
  `DOMDIR_examples`, where the "DOMDIR" is the name of the directory under
  `domains/` dedicated to that problem domain (benchmark).


Documentation
-------------

The directory `doc/` contains sources for several components of documentation.

* `doc/norm/` : Document provide normative problem domain descriptions and
  competition rules. Building requires [LaTeX](http://www.latex-project.org).
* `doc/api/` : API manual. Building requires [Doxygen](http://www.doxygen.org).
* `doc/usersguide/` : User's Guide. Building requires [Sphinx](http://sphinx-doc.org).

If the appropriate tools are installed, as noted above, then go to the desired
directory and run `make`.


License
-------

This is free software released under the terms of [the BSD 3-Clause License]
(http://opensource.org/licenses/BSD-3-Clause).  There is no warranty; not even
for merchantability or fitness for a particular purpose.  Consult LICENSE for
copying conditions.
