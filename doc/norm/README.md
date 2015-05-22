This directory contains LaTeX sources for normative problem domain descriptions
and competition rules. We also collect past proposals and other documents about
logistics for relevant events.

Building
--------
There is a Makefile.  Besides providing for the usual `make` command, it also
defines the following commands.  Output is placed in `build/`

* `make date` : build, including in the file name "DRAFT" and the current
  timestamp (UTC)

* `make gitdate` : extends `make date` to include in the file name the first
  7 numbers of the current commit hash.

* `clean` : delete everything under `build/`.
