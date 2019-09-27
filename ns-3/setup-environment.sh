#!/bin/bash
set -e # abort on error

hg clone http://code.nsnam.org/ns-3-allinone

cd ns-3-allinone
./download.py

cd ns-3-dev/src
ln -s ../../../external-sync ./
