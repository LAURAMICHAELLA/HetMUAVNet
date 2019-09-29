#!/bin/bash
cd "$(dirname "$0")"
THISDIR="$PWD"

mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
make
make install'
