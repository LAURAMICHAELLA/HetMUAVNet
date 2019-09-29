#!/bin/bash
set -e # abort on error

cd ../../ns-3-allinone/ns-3.29/src
ln -s ../../../external-sync ./

export PATH=/HetMUAVNet/HetMUAVNet/install/bin:/ns-3-allinone/ns-3.29/build/src/external-sync/examples:$PATH
export LD_LIBRARY_PATH=/ns-3-allinone/ns-3.29/build/lib
