Topical README files are provided in some directories.  For example,
`domains/dubins_traffic/README.md`.

Summary of organization
-----------------------

* `doc/` : Documentation sources, including benchmark APIs, the User's Guide,
  and competition rules.

* `domains/` : Problem domains, e.g., `domains/integrator_chains/` contains code
  for the scaling chains of integrators setting.

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
