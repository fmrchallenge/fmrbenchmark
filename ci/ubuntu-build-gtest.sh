#!/bin/sh -e
mkdir gtest-build
cd gtest-build
cmake /usr/src/googletest
make
sudo make install
